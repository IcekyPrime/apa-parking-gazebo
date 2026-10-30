from setuptools import find_packages, setup

package_name = 'risk_assessment'

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
    description='Collision risk assessment in the map frame',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'risk_node = risk_assessment.risk_node:main',
        ],
    },
)
