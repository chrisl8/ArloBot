import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'arlobot_ros'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name), ['arlobot.urdf']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chrisl8',
    maintainer_email='christen@lofland.net',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'propeller_node = arlobot_ros.propeller_node:main',
            'arlobot_teleop_key = arlobot_ros.arlobot_teleop_key:main',
            'OdometryPublisher = arlobot_ros.OdometryPublisher:main'
        ],
    },
)
