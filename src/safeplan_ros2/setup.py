from setuptools import find_packages, setup

package_name = 'safeplan_ros2'

from setuptools import setup

package_name = 'safe_plan_benchmark'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@example.com',
    description='ROS 2 benchmark planner wrapper',
    license='MIT',
    data_files=[
        ('share/' + package_name + '/launch', ['launch/safe_planner_launch.py']),
        ('share/' + package_name + '/config', ['config/algos.yaml']),
        ('share/' + package_name, ['resource/' + package_name]),
    ],
    entry_points={
        'console_scripts': [
            'safe_planner_node = safe_plan_benchmark.safe_planner_node:main'
        ],
    },
)

