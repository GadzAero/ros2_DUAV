from setuptools import find_packages, setup
from glob import glob

package_name = 'drone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='step',
    maintainer_email='binon.stepane@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'write_MAV = drone.write_MAV:main',
            'analyse_CAM = drone.analyse_CAM:main',
            'send_fire_coor = drone.send_fire_coor:main',
        ],
    },
)
