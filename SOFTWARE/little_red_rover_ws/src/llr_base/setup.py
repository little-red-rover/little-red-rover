from glob import glob
from setuptools import find_packages, setup
import os

package_name = 'llr_base'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ], 
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Michael Crum',
    maintainer_email='michael@michael-crum.com',
    description='Base package for Little Red Rover (LLR)',
    license='GPL-3.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_plotter = llr_base.lidar_plotter:main'
        ],
    },
)
