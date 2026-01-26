import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'ainex_social_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hrs2025',
    maintainer_email='hrs2025@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wave_hand_node = ainex_social_robot.wave_hand_node:main',
            'shake_head_node = ainex_social_robot.shake_head_node:main',
            'combine_node = ainex_social_robot.combine_node:main',
        ],
    },
)
