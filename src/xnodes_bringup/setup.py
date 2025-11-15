from setuptools import setup

package_name = "xnodes_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/bringup.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Workspace Maintainers",
    maintainer_email="maintainers@example.com",
    description="Bringup utilities for targeted build workflows.",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [],
    },
)
