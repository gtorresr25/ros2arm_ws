from setuptools import setup, find_packages

package_name = 'kinematics'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
