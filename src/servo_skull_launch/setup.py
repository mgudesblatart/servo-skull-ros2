from setuptools import find_packages, setup
import glob
import os

package_name = "servo_skull_launch"

launch_files = glob.glob(os.path.join('launch', '*.launch.py'))

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", launch_files),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="murray",
    maintainer_email="mgudesblatart@gmail.com",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [],
    },
)
