from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'drone_testbed'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dean Mascitti',
    maintainer_email='dean@todo.todo',
    description='Multi-drone testbed for distributed control algorithms',
    license='MIT',
    entry_points={
        'console_scripts': [
            'drone_node = drone_testbed.drone_node:main',
            'algorithm_manager = drone_testbed.algorithm_manager:main',
            'sim_visualizer = drone_testbed.sim_visualizer:main',
        ],
    },
)
