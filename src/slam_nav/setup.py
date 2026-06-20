from setuptools import setup

package_name = "slam_nav"

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
    description="Super-light 2D occupancy-grid SLAM + (later) navigation.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "nav_node = slam_nav.nav_node:main",
        ],
    },
)
