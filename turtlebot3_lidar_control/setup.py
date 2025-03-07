from glob import glob
from setuptools import find_packages, setup

package_name = 'turtlebot3_lidar_control'

setup(
    name=package_name,
    version='0.0.1',  # ปรับเวอร์ชันให้เหมาะสม
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # รองรับไฟล์ launch
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yuki',
    maintainer_email='pattharajarin.p@gmail.com',
    description='TurtleBot3 Lidar Control package for obstacle detection and movement.',
    license='Apache License 2.0',  # เปลี่ยนเป็น license ที่เหมาะสม (เช่น Apache 2.0)
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_reader = turtlebot3_lidar_control.lidar_reader:main',
            'turtlebot_controller = turtlebot3_lidar_control.turtlebot_controller:main',
            'obstacle_detector = turtlebot3_lidar_control.obstacle_detector:main',  # เพิ่ม Node ใหม่
        ],
    },
)
