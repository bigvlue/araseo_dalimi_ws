from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'araseo_hud'

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
    maintainer_email='intobinary@gmail.com',
    description='HUD overlay node for ARASEO-DALIMI',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hud_overlay = araseo_hud.hud_overlay_node:main',
        ],
    },
)
