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
        ]),
        ('share/' + package_name + '/launch', ['launch/dummy_algorithm.launch.py']),
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
        ],
    },
)
