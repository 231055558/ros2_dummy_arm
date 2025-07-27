from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gazebo_rviz_sync_demo'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 包含世界文件
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        # 包含启动文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # 包含 RViz 配置
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
        # 包含配置文件
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='li',
    maintainer_email='231055558@qq.com',
    description='Gazebo 和 RViz 同步演示包',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cube_synchronizer = gazebo_rviz_sync_demo.cube_synchronizer:main',
            'cube_controller = gazebo_rviz_sync_demo.cube_controller:main',
        ],
    },
)
