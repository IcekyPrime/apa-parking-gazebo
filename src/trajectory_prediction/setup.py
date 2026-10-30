from setuptools import find_packages, setup

package_name = 'trajectory_prediction'

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
    description='Transformer-based multi-object trajectory prediction',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'predictor_node = trajectory_prediction.predictor_node:main',
        ],
    },
)
