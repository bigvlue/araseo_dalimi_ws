from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'araseo_traffic_law'

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
    description='Korean traffic law compliance node for ARASEO-DALIMI',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_law_manager = araseo_traffic_law.traffic_law_manager_node:main',
        ],
    },
)
