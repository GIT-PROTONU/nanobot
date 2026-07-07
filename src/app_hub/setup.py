from setuptools import setup

package_name = "app_hub"

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
    description="Single-process host for web_control + oled_display + behavior "
                "(one executor, saves RAM).",
    license="MIT",
    entry_points={
        "console_scripts": [
            "app_hub = app_hub.hub:main",
        ],
    },
)
