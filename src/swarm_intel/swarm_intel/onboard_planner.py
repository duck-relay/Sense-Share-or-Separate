"""The route planner of a drone's on-board computer.

Every route a drone flies is computed by compute_route() on that drone's computer, from where the drone
is to its landing pad, on the drone's own map. Two ways to run it, with the same function, map and
random seed, so they return the same route:

  InlinePlanner   the headless twin (and the self-test): the route is ready in the same control step.
                  No simulated time passes while planning, so every mission is deterministic; the
                  planning time is still measured (wall clock) and reported.
  ProcessPlanner  the ROS 2 drone node (Gazebo): a separate planner process next to the flight loop,
                  like a companion computer next to the autopilot. While a route is being computed the
                  flight loop keeps running at full rate (EKF, give-way, safety layer, following the
                  current route) and the new route is swapped in when it is ready, so the real planning
                  delay of the drone's computer shows in the flight.

The drone asks for a route (submit), and polls for it every control step (poll). One route is computed
at a time; the drone decides what to do with it (agent.py: adopt it, keep its current route, or ask
again because its map changed meanwhile).
"""
import time

import numpy as np

from .grid_planner import plan_path


def compute_route(gm, start, goal, planner, seed, inflations):
    """One route on the drone's map: the first inflation that gives a route (wider clearance first).
    Returns (list of (x, y) or None, compute time in ms)."""
    rng = np.random.default_rng(seed)                 # RRT*'s samples (A* uses none)
    t0 = time.perf_counter()
    path = None
    for infl in inflations:
        path = plan_path(gm, start, goal, infl, planner=planner, rng=rng)
        if path is not None:
            break
    return path, 1000.0 * (time.perf_counter() - t0)


class InlinePlanner:
    """The route is computed at once, inside the control step that asks for it."""
    name = 'inline'

    def __init__(self):
        self._result = None

    def busy(self):
        return False

    def submit(self, gm, start, goal, planner, seed, inflations):
        self._result = compute_route(gm, start, goal, planner, seed, inflations)

    def poll(self):
        r, self._result = self._result, None
        return r

    def close(self):
        pass


def _warm_up():
    return True


def _ignore_sigint():
    """In the planner process: Ctrl+C in the terminal (or a launch stopping its process group) reaches every
    process of the group; the drone node closes its planner itself, so this one ignores it rather than
    printing a KeyboardInterrupt traceback into the logs."""
    import signal
    signal.signal(signal.SIGINT, signal.SIG_IGN)


class ProcessPlanner:
    """The route is computed in a separate process; the flight loop never waits for it. Should the
    planner process fail, the drone computes its routes inline instead (and says so once)."""
    name = 'process'

    def __init__(self, log=None):
        import concurrent.futures
        import multiprocessing
        self._log = log or (lambda msg: None)
        self._future = None
        self._inline = None
        try:
            # 'spawn': a fresh interpreter, never a fork of a process that runs ROS threads
            self._pool = concurrent.futures.ProcessPoolExecutor(
                max_workers=1, mp_context=multiprocessing.get_context('spawn'), initializer=_ignore_sigint)
            self._pool.submit(_warm_up)                # start the process (imports) before the first route
        except Exception as e:
            self._fail(e)

    def _fail(self, e):
        if self._inline is None:
            self._log(f'planner process unavailable ({type(e).__name__}: {e}) - planning inside the flight loop')
            self._inline = InlinePlanner()
            self._pool = None

    def busy(self):
        return self._future is not None

    def submit(self, gm, start, goal, planner, seed, inflations):
        if self._inline is None:
            try:
                # a frozen copy of the map: the drone keeps mapping while the route is computed
                self._future = self._pool.submit(compute_route, gm.snapshot(), tuple(start), tuple(goal), planner,
                                                 int(seed), tuple(inflations))
                return
            except Exception as e:
                self._fail(e)
        self._inline.submit(gm, start, goal, planner, seed, inflations)

    def poll(self):
        if self._inline is not None and self._future is None:
            return self._inline.poll()
        f = self._future
        if f is None or not f.done():
            return None
        self._future = None
        try:
            return f.result()
        except Exception as e:                         # the planner process died: no route this time
            self._fail(e)
            return None, 0.0

    def close(self):
        try:
            if self._pool is not None:
                self._pool.shutdown(wait=False, cancel_futures=True)
        except Exception:
            pass
