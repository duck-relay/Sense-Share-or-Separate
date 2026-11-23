"""swarm_intel: 5-drone navigation study (ROS 2 / Gazebo + headless twin).

Reproducibility. numpy's OpenBLAS picks its arithmetic kernel from the CPU model. Kernels round in the
last digits differently, and a mission can amplify such differences into a visibly different
trajectory (above all missions with the swarm radio). The results are computed on a reference machine
(a VirtualBox VM) in which OpenBLAS runs its generic Prescott (SSE3) kernel. That kernel runs on every
x86-64 CPU, so the package selects it before numpy loads, and the BLAS arithmetic of a mission is then
the same on any x86-64 computer. numpy's own vectorised functions and the C maths
library still depend on the CPU and the system, so on another computer some missions can still differ
in detail (README, section 1); on the same computer and setup every mission repeats exactly.

An OPENBLAS_CORETYPE you set yourself is always kept; SWARM_INTEL_NO_BLAS_PIN=1 turns the selection off.
"""
import os as _os
import sys as _sys

PUBLISHED_KERNEL = 'Prescott'       # OpenBLAS kernel of the published results (generic x86-64, SSE3)


def _pin_blas_kernel():
    if ('numpy' in _sys.modules or _os.environ.get('OPENBLAS_CORETYPE')
            or _os.environ.get('SWARM_INTEL_NO_BLAS_PIN')):
        return                      # too late (numpy already loaded), chosen by the user, or turned off
    try:
        with open('/proc/cpuinfo') as f:
            for line in f:
                if line.startswith('flags'):
                    if 'pni' in line.split(':', 1)[1].split():      # SSE3 ("pni"): every x86-64 CPU
                        _os.environ['OPENBLAS_CORETYPE'] = PUBLISHED_KERNEL
                    return
    except OSError:                 # not Linux: leave OpenBLAS's own choice
        pass


_pin_blas_kernel()


def blas_kernel_note():
    """One line for logs: which OpenBLAS kernel the results are computed with."""
    ct = _os.environ.get('OPENBLAS_CORETYPE')
    if ct == PUBLISHED_KERNEL:
        return f'OpenBLAS kernel {ct} (as for the published results)'
    if ct:
        return f'OpenBLAS kernel {ct} (set by you; the published results use {PUBLISHED_KERNEL})'
    return ('OpenBLAS kernel chosen by OpenBLAS (not pinned): missions can differ from the published ones in '
            'their details, above all those with the swarm radio; the statistics agree (README, section 1)')
