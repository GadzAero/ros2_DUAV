from setuptools import find_packages
from setuptools import setup

setup(
    name='wiimote_msgs',
    version='3.3.0',
    packages=find_packages(
        include=('wiimote_msgs', 'wiimote_msgs.*')),
)
