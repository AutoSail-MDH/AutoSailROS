#!/usr/bin/env python
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['camera',
              'ctrl',
              'motor_controller',
              'path_planner',
              'wind_sensor',
              'camera',
              'stm32'
              ],
    package_dir={'': 'src'}
)
setup(**setup_args)
