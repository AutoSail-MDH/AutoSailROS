#!/usr/bin/env python
from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['ctrl'],
    package_dir={'': 'src'},
    install_requires=[
        "numpy",
        "matplotlib"
    ]
)
setup(**setup_args)
