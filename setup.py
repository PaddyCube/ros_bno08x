import os
from glob import glob
from setuptools import setup

package_name = 'ros_bno08x'

setup(
    name=package_name,
    version='2.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma][xml]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Patrick Weber',
    maintainer_email='your@email.com',
    description='ROS2 node for Adafruit BNO085 IMU',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = ros_bno08x.talker:main'
        ],
    },
)