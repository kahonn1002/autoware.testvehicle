from glob import glob
from setuptools import find_packages, setup

package_name = "autoware_carmaker_interface"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="FSAI",
    maintainer_email="no@support.com",
    description="CarMaker adapter for Autoware",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "interface_node = autoware_carmaker_interface.interface_node:main",
        ],
    },
)

