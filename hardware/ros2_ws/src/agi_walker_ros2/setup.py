import os
from glob import glob
from setuptools import setup

package_name = "agi_walker_ros2"
package_share = os.path.join("share", package_name)

setup(
    name=package_name,
    version="4.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        (package_share, ["package.xml"]),
        (os.path.join(package_share, "launch"), glob("launch/*.py")),
        (os.path.join(package_share, "config"), glob("config/*.yaml")),
        (os.path.join(package_share, "urdf"), glob("urdf/*.urdf")),
        (os.path.join(package_share, "rviz"), glob("rviz/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AGI-Walker Team",
    maintainer_email="openneuro@example.com",
    description="ROS 2 integration for AGI-Walker",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "bridge_node = agi_walker_ros2.bridge_node:main",
        ],
    },
)
