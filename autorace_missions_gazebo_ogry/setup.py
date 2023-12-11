from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'autorace_missions_gazebo_ogry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mikhailp',
    maintainer_email='misha.pesh.00@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'intersection = autorace_missions_gazebo_ogry.intersection:main',
            'construction = autorace_missions_gazebo_ogry.construction:main',
            'parking = autorace_missions_gazebo_ogry.parking:main',
            'crossing = autorace_missions_gazebo_ogry.crossing:main',
            'tunnel = autorace_missions_gazebo_ogry.tunnel:main'
        ],
    },
)
