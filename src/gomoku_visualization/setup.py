from setuptools import find_packages, setup

package_name = 'gomoku_visualization'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'pygame',
        'rclpy',
        'std_msgs',
    ],
    zip_safe=True,
    maintainer='shuowen',
    maintainer_email='shuowen.li@tum.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gomoku_visualizer_node = gomoku_visualization.gomoku_visualizer_node:main',
        ],
    },
)
