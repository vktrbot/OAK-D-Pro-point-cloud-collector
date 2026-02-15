from setuptools import setup

package_name = "oak_cloud_collector"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", ["launch/snapshot.launch.py"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="vktrbot",
    maintainer_email="",
    description="ROS 2 PointCloud2 snapshot tool for OAK-D Pro: saves PLY + JSON metadata.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "snapshot = oak_cloud_collector.snapshot_node:main",
        ],
    },
)
