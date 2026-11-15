from glob import glob

from setuptools import find_packages
from setuptools import setup

PACKAGE_NAME = 'manipulation_isaac_sim'

setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml', 'README.md']),
        ('share/' + PACKAGE_NAME + '/config', glob('config/*.yaml')),
        ('share/' + PACKAGE_NAME + '/launch', glob('launch/*.launch.py')),
        ('share/' + PACKAGE_NAME + '/urdf', glob('urdf/*.xacro')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Dmitrii Ognev',
    maintainer_email='165507983+dmitriiognev@users.noreply.github.com',
    description='ROS 2 side of the Isaac Sim backend for the LWA 4P arm.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'make_isaac_urdf = manipulation_isaac_sim.make_isaac_urdf:main',
        ],
    },
)
