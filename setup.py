from setuptools import setup
import os
from glob import glob

package_name = 'ros2_pose_image_annotation'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='ROS 2 package for publishing random poses and annotated images',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_publisher = ros2_pose_image_annotation.pose_publisher:main',
            'image_publisher = ros2_pose_image_annotation.image_publisher:main',
            'annotate_node = ros2_pose_image_annotation.annotate_node:main',
        ],
    },
)
