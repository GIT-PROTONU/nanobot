from setuptools import setup

package_name = "sensor_hub"

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
    description="Single-process host for imu_driver + sys_monitor + wheel_odometry + "
                "lds_driver_py (one executor, saves RAM).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "sensor_hub = sensor_hub.hub:main",
        ],
    },
)
