from setuptools import setup
import os
from glob import glob

package_name = 'package_projet'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        # Install package.xml
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Your ROS 2 Package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'my_node = package_projet.my_node:main',
            'serveur = package_projet.serveur:main',
            'capteur_presence = package_projet.capteur_presence:main',
            'publisher_i2c_rand = package_projet.publisher_i2c_rand:main',
            'site = package_projet.site:main',
            'client = package_projet.client:main',
            'suscriber_basededonne = package_projet.suscriber_basededonne:main',
            'suscriber_i2c = package_projet.suscriber_i2c:main',
            'porte = package_projet.porte:main',
        ],
    },
)
