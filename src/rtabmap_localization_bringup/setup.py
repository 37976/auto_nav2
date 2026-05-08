from glob import glob
import os

from setuptools import find_packages, setup


package_name = "rtabmap_localization_bringup"


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="boxing",
    maintainer_email="clibang2022@163.com",
    description="Bringup package for multiple RTAB-Map localization pipelines.",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "xfeat_rtabmap_bridge = rtabmap_localization_bringup.xfeat_rtabmap_bridge:main",
            "xfeat_rgbd_odometry = rtabmap_localization_bringup.xfeat_rgbd_odometry:main",
        ],
    },
)
