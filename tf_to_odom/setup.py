from setuptools import setup
from setuptools import find_packages

package_name = 'tf_to_odom'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=[
        'setuptools',
        'rclpy',
        'tf2_ros',
        'nav_msgs',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='TF to Odometry Node',
    license='License Declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_to_odom_node = tf_to_odom.tf_to_odom_node:main'
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/tf_to_odom_launch.py']),
    ],
)
