from setuptools import find_packages, setup

package_name = 'autorace_vision_gazebo_ogry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (f'share/{package_name}/images',
            ['resource/intersection.png',
             'resource/construction.png',
             'resource/parking.png',
             'resource/crosswalk.png',
             'resource/parking.png',
             'resource/tunnel.png']),
        ('share/' + package_name, ['package.xml']),
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
            "sign_detector = autorace_vision_gazebo_ogry.sign_detector:main"
        ],
    },
)
