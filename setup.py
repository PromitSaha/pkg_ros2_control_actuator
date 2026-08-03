from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'control_actuator'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/control_actuator/launch', ['launch/ekf_launch.py']),
        ('share/control_actuator/config', ['config/ekf.yaml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        ('share/control_actuator/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'keyboard', 'pyserial'],
    zip_safe=True,
    maintainer='promit',
    maintainer_email='promit.mist@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_actuator_by_keyboard_executable = control_actuator.control_actuator_by_keyboard:main',
            'control_platform_by_keyboard_executable = control_actuator.control_platform_by_keyboard:main',
            'control_actuator_by_IMU_executable = control_actuator.control_actuator_by_IMU:main',
            'control_all_actuator_by_IMU_executable = control_actuator.control_all_actuator_by_IMU:main',
            'control_all_actuator_by_IMU_2_executable = control_actuator.control_all_actuator_by_IMU_2:main',
            'stewart_kinematics_node_executable = control_actuator.stewart_kinematics_node:main',
            'madgwick_node_executable = control_actuator.madgwick_node:main',
            'stabilizer_node_executable = control_actuator.stabilizer_node:main',
            'roll_pitch_printer_executable = control_actuator.roll_pitch_printer:main'
        ],
    },
)
