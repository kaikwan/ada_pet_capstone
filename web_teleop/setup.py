from setuptools import find_packages, setup
from glob import glob

package_name = "web_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
        ("share/" + package_name + "/config", glob("config/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kaikwan",
    maintainer_email="kaikwan@cs.washington.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    # tests_require=['pytest'],
    entry_points={
        "console_scripts": ["align_to_aruco = web_teleop.align_to_aruco:main"],
    },
)
