from setuptools import find_packages, setup
from glob import glob
import os

package_name = "sim"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Maintainer",
    maintainer_email="maintainer@example.com",
    description="2D ROS 2 kinematic bicycle simulator",
    license="Apache-2.0",
    entry_points={"console_scripts":
        ["sim = sim.simulator_node:main"]},
)
