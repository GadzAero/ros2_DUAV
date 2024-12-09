from setuptools import find_packages, setup

package_name = 'DUAV'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','rclpy'],
    zip_safe=True,
    maintainer='step',
    maintainer_email='step@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'write_MAV = DUAV.write_MAV:main',
            'pub_MAV = DUAV.pub_MAV:main',
            'create_path = DUAV.create_path:main',
            'analyse_CAM = DUAV.analyse_CAM:main',
            'determine_plane_state = DUAV.determine_plane_state:main',
            'select_mission_type = DUAV.select_mission_type:main',
            'create_costmap = DUAV.create_costmap:main',
        ],
    },
)
