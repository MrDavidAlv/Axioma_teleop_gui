from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'axioma_teleop_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Mario David Alvarez Vallejo',
    maintainer_email='ing.marioalvarezvallejo@gmail.com',
    author='Mario David Alvarez Vallejo',
    author_email='ing.marioalvarezvallejo@gmail.com',
    description='Modular teleoperation GUI for ROS 2 with multiple control modes. '
                'Initially developed for the Axioma robot but adaptable to any robot '
                'using geometry_msgs/Twist.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'teleop_gui = axioma_teleop_gui.main:main',
        ],
    },
)

