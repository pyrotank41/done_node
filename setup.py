from setuptools import setup
import os
from glob import glob

package_name = 'drone_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='karan',
    maintainer_email='pyrotank41@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'track_node = drone_tracker.track_node:main_entrypoint',
            'gimbal_tracker_node = drone_tracker.gimble_tracker_node:main',
            'drone_node = drone_tracker.drone_node:main',
            
        ],
    },
)
