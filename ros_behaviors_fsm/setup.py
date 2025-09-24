from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ros_behaviors_fsm'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'launch'), glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='benricket',
    maintainer_email='bricket@olin.edu',
    description='Wall following behavior for Neato robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simple_teleop = ros_behaviors_fsm.simple_teleop:main',
            'wall_identify_follow = ros_behaviors_fsm.wall_identify_follow:main'
        ],
    },
)
