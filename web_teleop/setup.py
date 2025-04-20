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
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hello-robot",
    maintainer_email="by1459@cs.washington.edu",
    description="TODO: Package description",
    license="TODO: License declaration",
    # tests_require=['pytest'],
    entry_points={
        "console_scripts": [],
    },
)
