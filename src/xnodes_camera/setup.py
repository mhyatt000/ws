from setuptools import setup

package_name = "xnodes_camera"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/camera.launch.py"]),
        ("share/" + package_name + "/config", ["config/camera.yaml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="Workspace Maintainers",
    maintainer_email="maintainers@example.com",
    description="Self-contained camera node configuration for testing targeted builds.",
    license="Apache License 2.0",
    entry_points={
        "console_scripts": [
            "camera_node = xnodes_camera.camera_node:main",
        ],
    },
)
