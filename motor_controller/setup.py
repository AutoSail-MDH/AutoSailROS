#!/usr/bin/env python
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['package_name'],
    package_dir={'': 'src'},
    install_requires=[
        "serial"
    ]
)
setup(**setup_args)