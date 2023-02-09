from setuptools import setup
from glob import glob

package_name = 'ME465_Lab2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*launch.py')),
        ('share/' + package_name, glob("config/*.yaml")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='todo',
    maintainer_email='todo@todo.todo',
    description='ME465 Lab 2',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lab2_node = ME465_Lab2.lab2_node:main',
            'detection_node = ME465_Lab2.detection_node:main',
            'plot_node = ME465_Lab2.plot_node:main',
            'save_data_node = ME465_Lab2.save_data_node:main'
        ],
    },
)
