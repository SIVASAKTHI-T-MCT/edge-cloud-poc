import os
from glob import glob
from setuptools import setup

package_name = 'handoff_edge'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sivasakthi',
    maintainer_email='you@example.com',
    description='Two-arm handoff edge layer',
    license='MIT',
    entry_points={
        'console_scripts': [
            'arm_node = handoff_edge.arm_node:main',
            'zone_monitor = handoff_edge.zone_monitor:main',
        ],
    },
)
