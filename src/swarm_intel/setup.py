from glob import glob
from setuptools import setup

package_name = 'swarm_intel'

setup(
    name=package_name,
    version='1.8.5',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='Rajat Verma',
    maintainer_email='r010702v@gmail.com',
    description='Drone navigation study: 12 navigation configurations and mixed fleets (EKF on every drone) vs the '
                '3-point camera + VIO baseline',
    license='GPL-3.0-or-later',
    entry_points={
        'console_scripts': [
            'world_node = swarm_intel.world_node:main',
            'drone_node = swarm_intel.drone_node:main',
            'dashboard = swarm_intel.dashboard:main',
            'compare = swarm_intel.compare:main',
            'batch = swarm_intel.batch:main',
            'study = swarm_intel.study:main',
            'planner_bench = swarm_intel.planner_bench:main',
            'selftest = swarm_intel.selftest:main',
            'gazebo_study = swarm_intel.gazebo_study:main',
            'validate = swarm_intel.validate:main',
            'paper_figures = swarm_intel.paper_figures:main',
            'reproduce = swarm_intel.reproduce:main',
            'controls = swarm_intel.controls:main',
            'menu = swarm_intel.menu:main',
            'generate_world = swarm_intel.world_gen:main',
        ],
    },
)
