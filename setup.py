from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'warehouse_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
            (
        'share/ament_index/resource_index/packages',
        ['resource/' + package_name],
    ),
    (
        'share/' + package_name,
        ['package.xml'],
    ),
    (
        os.path.join('share', package_name, 'launch'),
        glob('launch/*.launch.py'),
    ),
    (
        os.path.join('share', package_name, 'urdf'),
        glob('urdf/*'),
    ),
    (
        os.path.join('share', package_name, 'worlds'),
        glob('worlds/*'),
    ),
    (
        os.path.join('share', package_name, 'config'),
        glob('config/*'),
    ),
    (
        os.path.join('share', package_name, 'meshes'),
        glob('meshes/*'),
    ),
        ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jiawei',
    maintainer_email='jiawei@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'obstacle_avoidance = warehouse_robot.obstacle_avoidance:main',
            'twist_converter = warehouse_robot.twist_converter:main',
        ],
    },
)
