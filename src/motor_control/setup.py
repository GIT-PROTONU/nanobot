from setuptools import setup

package_name = "motor_control"

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
    description="cmd_vel -> PCA9685 differential drive.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "motor_node = motor_control.motor_node:main",
        ],
    },
)
