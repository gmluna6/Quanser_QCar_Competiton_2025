from setuptools import find_packages
from setuptools import setup

setup(
    name='qcar2_navigation',
    version='0.0.0',
    packages=find_packages(
        include=('qcar2_navigation', 'qcar2_navigation.*')),
)
