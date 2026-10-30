from glob import glob
import os

from setuptools import setup

package_name = 'parking_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student C',
    maintainer_email='studentc@example.com',
    description='泊车规划控制层：车位检测 + Hybrid A* + Reeds-Shepp + 控制',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner_node = parking_planner.planner_node:main',
            'cmd_mux = parking_planner.cmd_mux:main',
            'make_layout = parking_planner.make_layout:main',
            'parking_eval = parking_planner.parking_eval:main',
            'selfcheck = parking_planner.selfcheck:main',
        ],
    },
)
