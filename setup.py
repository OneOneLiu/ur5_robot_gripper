import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

from setuptools import find_packages, setup

package_name = 'ur5_robot_gripper'

def package_files(directory):
    paths = []
    for path, _, files in os.walk(directory):
        for file in files:
            paths.append((os.path.join('share', package_name, path), [os.path.join(path, file)]))
    return paths

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ] + package_files('urdf') + package_files('meshes'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='daohui.liu@mail.utoronto.ca',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sim_control = ur5_robot_gripper.robotiq.sim_control:main',
        ],
    },
)
