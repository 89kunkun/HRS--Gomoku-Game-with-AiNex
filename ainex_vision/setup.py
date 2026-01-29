import os
from setuptools import find_packages, setup

package_name = 'ainex_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'),
            ['ainex_vision/camera.yaml']),
    ],
    install_requires=['setuptools'],
    include_package_data=True,
    package_data={'ainex_vision': ['camera.yaml']},
    zip_safe=True,
    maintainer='Wenlan Shen',
    maintainer_email='wenlan.shen@tum.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'camera_sub = ainex_vision.camera_sub:main',
          'tutorial_2 = ainex_vision.tutorial_2:main',
          'tutorial_3 = ainex_vision.tutorial_3:main',
          'tutorial_4 = ainex_vision.tutorial_4:main',
          'calibration= ainex_vision.calibration:main',
          'calibrate_camera= ainex_vision.calibrate_camera:main',
          'undistort_node= ainex_vision.undistort_node:main',
          'face_detection_node= ainex_vision.face_detection_node:main',
          'aruco_detector= ainex_vision.aruco_detector:main',
          'aruco_board_location = ainex_vision.aruco_board_location:main',
          'pieces_detection = ainex_vision.pieces_detection:main',
          'arrayoutput = ainex_vision.arrayoutput:main',
          'board_to_base = ainex_vision.board_to_base:main',
          'sym_point = ainex_vision.sym_point:main',

        ],
    },
)
