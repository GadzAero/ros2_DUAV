from setuptools import find_packages, setup
from glob import glob

package_name = 'popeye'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*.launch.py')),
        # ('lib/python3.10/site-packages/' + package_name, ['popeye/utils/FSM_utils.py']),
        # ('lib/python3.10/site-packages/' + package_name, ['popeye/utils/MAV_utils.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='step',
    maintainer_email='binon.stepane@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    entry_points={
        'console_scripts': [
            'MAV_manager = popeye.MAV_manager:main',
            'FSM_interface = popeye.FSM_interface:main',
        ],
    },
)
