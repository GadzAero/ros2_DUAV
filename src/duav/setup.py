from setuptools import find_packages, setup
from glob import glob

package_name = 'duav'

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
            'write_MAV = duav.write_MAV:main',
            'pub_MAV = duav.pub_MAV:main',
            'create_path = duav.create_path:main',
            'analyse_CAM = duav.analyse_CAM:main',
            'determine_plane_state = duav.determine_plane_state:main',
            'select_mission_type = duav.select_mission_type:main',
            'create_costmap = duav.create_costmap:main',
        ],
    },
)
