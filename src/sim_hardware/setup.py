from setuptools import setup

package_name = "sim_hardware"

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
    description="Dev-PC-only Gazebo hardware stand-in (lidar/IMU/encoders/telemetry) for dev/prod ROS parity.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "sim_bridge_node = sim_hardware.sim_bridge_node:main",
            "map_bridge_node = sim_hardware.map_bridge_node:main",
        ],
    },
)
