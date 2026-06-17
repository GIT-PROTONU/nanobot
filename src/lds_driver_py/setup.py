from setuptools import setup

package_name = "lds_driver_py"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ib",
    maintainer_email="ib.elfaramawy@gmail.com",
    description="Roborock LDS02RR / Neato lidar serial driver (Python) -> sensor_msgs/LaserScan.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "lds_node = lds_driver_py.lds_node:main",
        ],
    },
)
