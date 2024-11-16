from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bumperbot_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'launch'),glob('launch/*.py')),
        (os.path.join('share', package_name, 'rviz2'), glob('rviz2/*')),
        (os.path.join('share', package_name, 'urdf'),  glob('urdf/*')),
        (os.path.join('share', package_name, 'world'), glob('world/*')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jlukas',
    maintainer_email='lukasjohnnytechmon@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'twist_relay = bumperbot_bringup.twist_relay:main',
            'safety_stop = bumperbot_bringup.safety_stop:main',
            'imu_republisher = bumperbot_bringup.imu_republisher:main',

        ],
    },
)
