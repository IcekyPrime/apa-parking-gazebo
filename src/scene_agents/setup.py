from glob import glob
import os

from setuptools import setup

package_name = 'scene_agents'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student C',
    maintainer_email='studentc@example.com',
    description='场景干扰物：循环社会车辆 + 停入车位的干扰车 + 行人',
    license='MIT',
    entry_points={
        'console_scripts': [
            'traffic_driver = scene_agents.traffic_driver:main',
            'traffic_driver_b = scene_agents.traffic_driver_b:main',
            'parker = scene_agents.parker:main',
            'pedestrian_agent = scene_agents.pedestrian_agent:main',
        ],
    },
)
