from glob import glob
from setuptools import find_packages, setup

package_name = "autoware_carmaker_launch"
setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml", "README.md"]),
        ("share/" + package_name + "/launch", glob("launch/*launch.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/config/profiles", glob("config/profiles/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="FSAI",
    maintainer_email="no@support.com",
    description="CarMaker end-to-end launch, map generators and TestRun converter",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "generate_sprint_map = autoware_carmaker_launch.generate_sprint_map:main",
            "generate_iso_map = autoware_carmaker_launch.generate_iso_map:main",
            "testrun_to_autoware = autoware_carmaker_launch.testrun.cli:main",
        ]
    },
)
