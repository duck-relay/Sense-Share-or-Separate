"""What a result was computed with.

Every result folder (studies, validation, planner benchmark, batch runs, paper/) gets a meta.json: the model
version, the package version, the planner and give-way, the OpenBLAS kernel, numpy, Python and the CPU. The
model version decides whether results can be combined: paper_figures uses only folders of the current model
version (a single Gazebo run carries the same fields in its summary.json).
"""
import json
import os
import platform
from datetime import datetime

META = 'meta.json'


def cpu_name():
    try:
        with open('/proc/cpuinfo') as f:
            for ln in f:
                if ln.startswith('model name'):
                    return ln.split(':', 1)[1].strip()
    except OSError:
        pass
    return platform.processor() or platform.machine()


def package_version():
    """The installed package's version (colcon install), else the source tree's package.xml."""
    try:
        from importlib.metadata import version
        return version('swarm_intel')
    except Exception:
        pass
    try:
        import re
        here = os.path.dirname(os.path.abspath(__file__))
        with open(os.path.join(here, '..', 'package.xml')) as f:
            m = re.search(r'<version>([^<]+)</version>', f.read())
        return m.group(1) if m else None
    except OSError:
        return None


def run_info(**extra):
    import numpy as np
    from . import config as C
    info = {'model_version': C.MODEL_VERSION, 'package_version': package_version(),
            'planner': C.PLANNER, 'give_way': C.GIVE_WAY,
            'openblas_coretype': os.environ.get('OPENBLAS_CORETYPE') or 'chosen by OpenBLAS',
            'numpy': np.__version__, 'python': platform.python_version(), 'cpu': cpu_name(),
            'created': datetime.now().isoformat(timespec='seconds')}
    info.update(extra)
    return info


def write_meta(folder, **extra):
    """Write <folder>/meta.json; returns its content."""
    info = run_info(**extra)
    with open(os.path.join(folder, META), 'w') as f:
        json.dump(info, f, indent=2, default=str)
    return info


def read_meta(folder):
    try:
        with open(os.path.join(folder, META)) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def is_current(folder):
    """Made with the current model version? (folders without meta.json predate it: no)"""
    from . import config as C
    m = read_meta(folder)
    return bool(m) and m.get('model_version') == C.MODEL_VERSION
