from setuptools import setup

package_name = 'ros2_pose_image_annotation'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/pose_image_annotation_launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neo',
    maintainer_email='neo@example.com',
    description='ROS 2 package to annotate pose on map image and stream to Foxglove',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_to_image_publisher = ros2_pose_image_annotation.map_to_image_publisher:main',
            'gz_pose_filter = ros2_pose_image_annotation.gz_pose_filter:main',
            'annotate_node = ros2_pose_image_annotation.annotate_node:main',
        ],
    },
)
