from setuptools import setup

package_name = "oled_display"

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
    description="Status OLED (SSD1306) over I2C.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "display_node = oled_display.display_node:main",
        ],
    },
)
