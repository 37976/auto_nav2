from glob import glob

from setuptools import find_packages, setup

package_name = "nav_eval"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xu",
    maintainer_email="xu@example.com",
    description="Navigation evaluation logger for auto_nav2.",
    license="GPL-2.0-or-later",
    entry_points={
        "console_scripts": [
            "auto_goal_runner = nav_eval.auto_goal_runner:main",
            "nav_eval_node = nav_eval.nav_eval_node:main",
        ],
    },
)
