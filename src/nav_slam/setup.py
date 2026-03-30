
from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'nav_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'map'), glob('map/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='boxing/喵了个水蓝蓝',
    maintainer_email='clibang2022@163.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'astar = nav_slam.astar:main',
            'map_pub = nav_slam.map_pub:main',
            'odom_map_tf = nav_slam.odom_map_tf:main',
            'points_pub_map = nav_slam.points_pub_map:main',
            'dynamic_obstacle_publisher = nav_slam.dynamic_obstacle_publisher:main',
            'start_nav = nav_slam.start_nav:main',
            
        ],
    },
)
