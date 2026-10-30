from setuptools import find_packages, setup

package_name = 'detection_tracking'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='es',
    maintainer_email='es@todo.todo',
    description='LiDAR-based multi-object detection and tracking',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'detection_node = detection_tracking.detection_node:main',
        ],
    },
)
