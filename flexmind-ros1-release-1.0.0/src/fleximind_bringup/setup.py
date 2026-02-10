#!/usr/bin/env python
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=["fleximind_bringup"],  # 与包的Python模块名称一致
    package_dir={"": "src"},  # 指定Python模块在src目录下
)

setup(**d)
