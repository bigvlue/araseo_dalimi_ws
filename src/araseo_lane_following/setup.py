from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'araseo_lane_following'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ARASEO-DALIMI',
    maintainer_email='todo@todo.com',
    description='Lane following node for ARASEO-DALIMI',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_follower = araseo_lane_following.lane_follower_node:main',
        ],
    },
)
