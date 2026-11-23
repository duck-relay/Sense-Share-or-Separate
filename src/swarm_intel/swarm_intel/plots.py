"""Shared matplotlib styling for the comparison graphs."""
from . import config as C

SURFACE = '#fcfcfb'
INK = '#0b0b0b'
INK2 = '#52514e'
MUTED = '#8a8984'
GRID = '#e7e6e2'
CAT_A = '#4a3aa7'      # categorical slot (run / config A) - violet, so blue keeps meaning 'better'
CAT_B = '#eb6834'      # categorical slot (run / config B) - orange
BETTER = '#2a78d6'     # better than the baseline (diverging pair: blue <-> red)
WORSE = '#e34948'      # worse than the baseline
SAME = '#b9b8b2'       # about the same
MIXED = '#5b6b7f'      # better on one axis, worse on the other
NEUTRAL = '#f0efec'    # diverging midpoint
IDEAL = '#2fa84f'      # reference route (A* on board before take-off): green, as in Gazebo
# a drone's flown path is drawn in the drone's own colour (config.DRONE_COLORS), as its trail in Gazebo
TREE = '#cfe3c8'
TREE_EDGE = '#8fb884'
PINE = '#3d4a3f'


def setup():
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        'figure.facecolor': SURFACE, 'axes.facecolor': SURFACE, 'savefig.facecolor': SURFACE,
        'axes.edgecolor': GRID, 'axes.linewidth': 0.8, 'axes.labelcolor': INK2, 'axes.titlecolor': INK,
        'axes.titlesize': 11.5, 'axes.titleweight': 'bold', 'axes.titlelocation': 'left', 'axes.labelsize': 9.5,
        'xtick.color': INK2, 'ytick.color': INK2, 'xtick.labelsize': 8.5, 'ytick.labelsize': 8.5,
        'axes.grid': True, 'grid.color': GRID, 'grid.linewidth': 0.7, 'grid.linestyle': '-',
        'axes.spines.top': False, 'axes.spines.right': False, 'legend.frameon': False,
        'legend.fontsize': 8.5, 'font.size': 9.5, 'lines.linewidth': 2.0,
        'lines.solid_capstyle': 'round',
    })
    return plt


def draw_arena(ax, show_unknown=True, drones=True):
    """The forest, and every drone's start (circle) and goal (star) in the drone's colour."""
    import matplotlib.patches as mp
    ax.add_patch(mp.Rectangle((-C.ARENA_HALF, -C.ARENA_HALF), 2 * C.ARENA_HALF, 2 * C.ARENA_HALF,
                              fill=False, ec=MUTED, lw=0.8))
    for (x, y) in C.LANDMARK_TREES:
        ax.add_patch(mp.Circle((x, y), C.TREE_CROWN_R, fc=TREE, ec=TREE_EDGE, lw=0.8, zorder=1))
    if show_unknown:
        for (x, y) in C.UNKNOWN_PINES:
            ax.add_patch(mp.Circle((x, y), C.PINE_PLAN_R, fc='none', ec=PINE, lw=1.4, zorder=1))
            ax.plot([x], [y], marker='x', ms=4, color=PINE, mew=1.2, zorder=1)
    if drones:
        for d in C.DRONES:
            ax.plot(*d.spawn, marker='o', ms=8, mfc=SURFACE, mec=d.color, mew=2.0, zorder=6)
            ax.plot(*d.goal, marker='*', ms=13, mfc=d.color, mec=INK, mew=0.6, zorder=7)
            ax.annotate(str(d.id), d.spawn, xytext=(0, 0), textcoords='offset points', ha='center', va='center',
                        fontsize=6.5, fontweight='bold', color=INK, zorder=8)
    ax.set_xlim(-C.ARENA_HALF - 0.4, C.ARENA_HALF + 0.4)
    ax.set_ylim(-C.ARENA_HALF - 0.4, C.ARENA_HALF + 0.4)
    ax.set_aspect('equal')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')


def arena_legend_handles(flown=False):
    from matplotlib.lines import Line2D
    h = [Line2D([], [], color=IDEAL, lw=4, alpha=0.5, label='Reference routes (A* before take-off)')]
    if flown:
        h += [Line2D([], [], color=d.color, lw=2.0, label=f'{d.name} flown') for d in C.DRONES]
    h += [Line2D([], [], marker='o', ls='none', mfc=TREE, mec=TREE_EDGE, ms=8, label='Known tree'),
          Line2D([], [], marker='o', ls='none', mfc='none', mec=PINE, ms=8, label='Unknown pine'),
          Line2D([], [], marker='o', ls='none', mfc=SURFACE, mec=MUTED, mew=2, ms=7, label='Start (no. = drone)'),
          Line2D([], [], marker='*', ls='none', mfc=MUTED, mec=INK, ms=11, label='Goal (drone colour)')]
    return h


CODES = {'cam3_sensor': 'C3+V', 'cam2': 'C2', 'sensor': 'V', 'swarm': 'W', 'cam2_sensor': 'C2+V',
         'cam2_swarm': 'C2+W', 'sensor_swarm': 'V+W', 'all': 'C2+V+W', 'cam3': 'C3', 'cam3_swarm': 'C3+W',
         'cam3_sensor_swarm': 'C3+V+W', 'cam3_sensor_layers': 'C3+V+L'}
CODE_KEY = 'C2/C3: 2-/3-point camera, V: VIO velocity, W: swarm radio, L: fixed altitude layers (no radio)'


def code(key):
    """Compact code of a configuration (C3+V = baseline) or mixed fleet (k2+W = 2 cameras, swarm)."""
    if key in CODES:
        return CODES[key]
    if key in C.FLEETS:
        f = C.FLEETS[key]
        return f'k{f.n_cameras}' + ('+W' if f.swarm else '')
    return key


def short_label(key):
    """Two-line configuration name for crowded axes."""
    fixed = {'all': 'All-in-one', 'cam2': '2-point\ncamera', 'sensor': 'VIO\nonly', 'swarm': 'Swarm\nonly',
             'sensor_swarm': 'VIO\n+ swarm', 'cam3': '3-point\ncamera', 'cam3_sensor_layers': 'Baseline +\nfixed layers',
             'cam3_sensor_swarm': '3-pt camera\n+ VIO\n+ swarm'}
    if key in fixed:
        return fixed[key]
    if key in C.FLEETS:
        f = C.FLEETS[key]
        return f'{f.n_cameras} of 5 with\ncamera' + ('\n+ swarm' if f.swarm else '')
    parts = C.NAV[key].parts.replace('-point camera', '-pt camera').split(' + ')
    return '\n+ '.join(parts)


def footer(fig, text):
    fig.text(0.012, 0.008, text, color=MUTED, fontsize=7.5, ha='left', va='bottom')
