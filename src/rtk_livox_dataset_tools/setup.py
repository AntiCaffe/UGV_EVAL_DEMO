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
    description="Field collection and calibration tools for Livox Avia plus RTK GNSS datasets.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "c099_udp_bridge = rtk_livox_dataset_tools.c099_udp_bridge:main",
            "rtk_status_monitor = rtk_livox_dataset_tools.rtk_status_monitor:main",
            "lidar_pose_calibrator = rtk_livox_dataset_tools.lidar_pose_calibrator:main",
            "online_lidar_pose_calibrator = rtk_livox_dataset_tools.online_lidar_pose_calibrator:main",
            "gt_transformer = rtk_livox_dataset_tools.gt_transformer:main",
            "opencl_dataset_exporter = rtk_livox_dataset_tools.opencl_dataset_exporter:main",
            "opencl_dataset_visualizer = rtk_livox_dataset_tools.opencl_dataset_visualizer:main",
            "rtk_livox_visualizer = rtk_livox_dataset_tools.rtk_livox_visualizer:main",
        ],
    },
)
