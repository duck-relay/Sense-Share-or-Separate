"""Clean stop of the ROS nodes (world_node, drone_node).

ros2 launch stops a node with SIGINT (later SIGTERM, then SIGKILL). Python's default SIGINT handler raises
KeyboardInterrupt wherever the main thread is at that moment - also inside rclpy's C++ code while it
converts a received message, which then fails with
    RuntimeError: Unable to convert call argument to Python object
and the node "dies" with exit code 1 although the mission had finished normally. Here a stop signal only
sets a flag; the spin loop checks it between callbacks and ends normally. A second signal (Ctrl+C twice)
still interrupts at once.
"""
import signal


class StopSignal:
    """Truthy once SIGINT or SIGTERM has arrived."""

    def __init__(self, signals=(signal.SIGINT, signal.SIGTERM)):
        self.signum = None
        for sig in signals:
            try:
                signal.signal(sig, self._handler)
            except (ValueError, OSError):      # not the main thread: keep the default behaviour
                pass

    def _handler(self, signum, frame):
        if self.signum is not None:            # second signal: stop now
            raise KeyboardInterrupt
        self.signum = signum

    def __bool__(self):
        return self.signum is not None


def is_shutdown_error(exc, stop, rclpy):
    """True if an exception from spin_once only reflects that the node is being stopped."""
    if type(exc).__name__ == 'ExternalShutdownException':
        return True
    try:
        ok = rclpy.ok()
    except Exception:
        ok = False
    return bool(stop) or not ok
