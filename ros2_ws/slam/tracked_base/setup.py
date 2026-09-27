from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tracked_base'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='openEuler',
    maintainer_email='sunqianhao@stu.scu.edu.cn',
    description='STM32 tracked robot serial bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'stm32_bridge = tracked_base.stm32_bridge:main',
        ],
    },
)
