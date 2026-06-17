from setuptools import setup

package_name = "sys_monitor"

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
    description="Board health monitor: CPU/mem/temp/disk on /diagnostics.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "monitor_node = sys_monitor.monitor_node:main",
        ],
    },
)
