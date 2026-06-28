import os
from glob import glob
from setuptools import setup

package_name = "web_control"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "web"), glob("web/*")),
        # The skill library (capabilities as self-documenting markdown). Installed under the
        # package share so the node finds them; also resolvable from the source tree.
        (os.path.join("share", package_name, "skills"), glob("skills/*.md")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ib",
    maintainer_email="ib.elfaramawy@gmail.com",
    description="rosbridge + static web control page.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "web_server = web_control.web_server:main",
        ],
    },
)
