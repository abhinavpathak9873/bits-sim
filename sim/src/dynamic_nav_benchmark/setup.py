from setuptools import setup

package_name = 'dynamic_nav_benchmark'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/experiments', [
            'experiments/tb3_mall.yaml',
            'experiments/tb3_airport.yaml',
            'experiments/tb4_mall.yaml',
            'experiments/tb4_airport.yaml',
            'experiments/baseline_go_to_goal_mall.yaml',
            'experiments/baseline_bug2_mall.yaml',
            'experiments/baseline_vfh_mall.yaml',
            'experiments/baseline_follow_gap_mall.yaml',
            'experiments/baseline_astar_mall.yaml',
            'experiments/baseline_vfh_airport.yaml',
            'experiments/baseline_astar_airport.yaml',
            'experiments/baseline_compare_mall.yaml',
        ]),
        ('share/' + package_name + '/launch', [
            'launch/dummy_algorithm.launch.py',
            'launch/baseline_go_to_goal.launch.py',
            'launch/baseline_bug2.launch.py',
            'launch/baseline_vfh.launch.py',
            'launch/baseline_follow_gap.launch.py',
            'launch/baseline_astar.launch.py',
            'launch/goal_marker.launch.py',
        ]),
    ],
    install_requires=['setuptools', 'PyYAML', 'matplotlib', 'numpy'],
    zip_safe=True,
    maintainer='Dynamic Nav Research Team',
    maintainer_email='research@example.com',
    description='Benchmark runner and metrics for dynamic navigation simulations.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'experiment_runner = dynamic_nav_benchmark.experiment_runner:main',
            'batch_runner = dynamic_nav_benchmark.batch_runner:main',
            'dummy_algorithm = dynamic_nav_benchmark.dummy_algorithm:main',
            'baseline_go_to_goal = dynamic_nav_benchmark.baseline_algorithms:main_go_to_goal',
            'baseline_bug2 = dynamic_nav_benchmark.baseline_algorithms:main_bug2',
            'baseline_vfh = dynamic_nav_benchmark.baseline_algorithms:main_vfh',
            'baseline_follow_gap = dynamic_nav_benchmark.baseline_algorithms:main_follow_gap',
            'baseline_astar = dynamic_nav_benchmark.baseline_algorithms:main_astar',
            'goal_marker = dynamic_nav_benchmark.goal_marker:main',
        ],
    },
)
