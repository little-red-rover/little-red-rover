from setuptools import find_packages, setup
import os
from glob import glob

package_name = "little_red_rover"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*")),
        (
            os.path.join("share", package_name, "description"),
            [f for f in glob("description/*") if os.path.isfile(f)],
        ),
        (
            os.path.join("share", package_name, "description", "rover"),
            [f for f in glob("description/*/*") if os.path.isfile(f)],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Michael Crum",
    maintainer_email="michael@michael-crum.com",
    description="Drivers for the little red rover robotics platform",
    license="LGPL-2.1",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "base = little_red_rover.base:main",
            "odometry_publisher = little_red_rover.odometry_publisher:main",
            "hal = little_red_rover.hal:main",
        ],
    },
)
