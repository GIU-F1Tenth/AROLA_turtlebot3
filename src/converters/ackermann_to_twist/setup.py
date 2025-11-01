from setuptools import setup
import os
from glob import glob

package_name = 'ackermann_to_twist'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Ackermann to Twist converter for differential drive robots',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ackermann_to_twist_node = ackermann_to_twist.ackermann_to_twist_node:main',
        ],
    },
)
