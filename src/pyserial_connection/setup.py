from setuptools import find_packages, setup

package_name = 'pyserial_connection'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial', 'rclpy'],
    zip_safe=True,
    maintainer='youssef',
    maintainer_email='youssef@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller = pyserial_connection.motor_controller:main',
            'motor_receiver = pyserial_connection.motor_receiver:main',
            'reader = pyserial_connection.reader:main',
            'pyserial_simulator = pyserial_connection.pyserial_simulator:main',
            'readerandsender = pyserial_connection.readerandsender:main'
        ],
    },
)
