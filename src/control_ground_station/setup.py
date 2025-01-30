from setuptools import find_packages, setup
from glob import glob

package_name = 'control_ground_station'

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
            # 'drone_menu_control = ihm.drone_menu_control:main',
            'MAV_manager_popeye = control_ground_station.MAV_manager_popeye:main',
            'MAV_manager_olive = control_ground_station.MAV_manager_olive:main',
        ],
    },
)
