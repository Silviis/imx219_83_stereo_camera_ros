import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'imx219_83_stereo_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'calibration'), glob(os.path.join('calibration', '*.yaml'))),


    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eetu Silvennoinen',
    maintainer_email='eetu.silvennoinen@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
