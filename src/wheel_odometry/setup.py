from setuptools import setup

package_name = "wheel_odometry"

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
    description="Wheel encoder odometry from the ESP32 coprocessor (/wheel_ticks).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "encoder_node = wheel_odometry.encoder_node:main",
        ],
    },
)
