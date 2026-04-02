from glob import glob
from setuptools import setup

package_name = "dashgo_web_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/web", glob("web/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="xu",
    maintainer_email="xu@example.com",
    description="Mobile web control panel for Dashgo real robot navigation.",
    license="GPL-2.0-or-later",
    entry_points={
        "console_scripts": [
            "web_control_node = dashgo_web_control.web_control_node:main",
            "show_web_qr = dashgo_web_control.show_web_qr:main",
            "show_web_qr_popup = dashgo_web_control.show_web_qr_popup:main",
            "dashgo_hotspot = dashgo_web_control.hotspot_manager:main",
        ],
    },
)
