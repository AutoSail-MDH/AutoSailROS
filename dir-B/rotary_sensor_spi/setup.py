#!/usr/bin/env python
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    scripts=['scripts'],
    packages=['rotary_sensor_spi'],
    package_dir={'': 'src'}
)

setup(**setup_args)