from setuptools import find_packages, setup

package_name = 'dasrobot_base_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='DASRobot',
    maintainer_email='das.developer@mail.ru',
    description='DASRobot base controller: serial, encoders → odom/tf',
    license='Apache-2.0', 
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dasrobot_serial = dasrobot_base_controller.dasrobot_serial_node:main',
            'dasrobot_odom = dasrobot_base_controller.dasrobot_odom_node:main',
            'dasrobot_cmd_convert = dasrobot_base_controller.dasrobot_cmd_converter_node:main'
        ],
    },
)
