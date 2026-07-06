from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'vision_test'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dev',
    maintainer_email='dev@example.com',
    description='Vision inference test for STM32MP257F-DK',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'vision_test_node = vision_test.bench_node:main',
        ],
    },
)