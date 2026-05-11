from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'kinematics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    package_data={package_name: ['armpi_ultra.urdf']},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/urdf', glob('urdf/*.urdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='andres',
    maintainer_email='andres.torres25g@gmail.com',
    description='Pure-Python IK/FK for ArmPi Ultra using ikpy and the robot URDF.',
    license='MIT',
    entry_points={
        'console_scripts': [],
    },
)
