from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'reactive_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='youssef',
    maintainer_email='yousefmakram27@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'reactive_nav_node = reactive_nav.reactive_nav_node:main',
            'sensor_fusion = reactive_nav.sensor_fusion:main',
            'pid_control = reactive_nav.pid_control:main',
            'keyboard_control = reactive_nav.keyboard_control:main',
            'teleop_controller = reactive_nav.teleop_controller:main',
            'serial_reader = reactive_nav.serial_reader:main',
            'camera_decision = reactive_nav.camera_decision:main'
        ],
    },
)
