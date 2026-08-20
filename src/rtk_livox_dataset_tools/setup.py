from glob import glob
from setuptools import setup

package_name = "rtk_livox_dataset_tools"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="AIV",
    maintainer_email="aiv@example.com",
    description="Bag postprocessing and visualization tools for Livox and RTK datasets.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "accumulated_bag_exporter = rtk_livox_dataset_tools.accumulated_bag_exporter:main",
            "rtk_livox_visualizer = rtk_livox_dataset_tools.rtk_livox_visualizer:main",
        ],
    },
)
