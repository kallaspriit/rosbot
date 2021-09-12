from setuptools import setup

package_name = 'odrive_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jiang Yue',
    maintainer_email='maze1024@gmail.com',
    description='ROS2 node for ODrive',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odrive_node = odrive_ros2.odrive_node:main'
        ],
    },
)
