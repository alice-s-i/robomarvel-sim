import os
from glob import glob

from setuptools import find_packages, setup

package_name = "webcam"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (os.path.join("share", package_name), ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Alisa",
    maintainer_email="alisasakva@gmail.com",
    description="Мост: видеопоток веб-камеры браузера в топик ROS 2",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "rtsp2ros = webcam.rtsp2ros:main",
        ],
    },
)
