from setuptools import setup

package_name = 'dynamic_nav_actors'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    maintainer='Dynamic Nav Research Team',
    maintainer_email='research@example.com',
    description='Dynamic obstacle spawning and control for navigation benchmarks.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'actor_manager = dynamic_nav_actors.actor_manager:main',
        ],
    },
)
