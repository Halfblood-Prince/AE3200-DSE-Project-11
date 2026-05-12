import os
from glob import glob
from pathlib import Path
from setuptools import setup

package_name = "ros_test"


def collect_data_files(source_dir, destination):
    source_path = Path(source_dir)
    if not source_path.exists():
        return []

    data_files = []
    for directory, _, filenames in os.walk(source_path):
        files = [str(Path(directory) / filename) for filename in filenames]
        if not files:
            continue

        relative_dir = Path(directory).relative_to(source_path)
        install_dir = Path("share") / package_name / destination / relative_dir
        data_files.append((str(install_dir), files))

    return data_files


setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md", "robot.sdf"]),
        (f"share/{package_name}/config", glob("config/*.config")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/rviz", glob("rviz/*.rviz")),
    ]
    + collect_data_files("website", "website"),
    install_requires=["setuptools", "Flask>=3.0,<4.0", "opencv-python>=4.8,<5.0"],
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Gazebo Harmonic lidar robot simulation with ROS 2 Jazzy SLAM Toolbox mapping.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "auto_drive = ros_test.auto_drive:main",
            "map_filter = ros_test.map_filter:main",
            "map_monitor = ros_test.map_monitor:main",
            "nav2_waypoint_explorer = ros_test.nav2_waypoint_explorer:main",
            "odom_to_tf = ros_test.odom_to_tf:main",
            "scan_to_chassis = ros_test.scan_to_chassis:main",
            "simple_mapper = ros_test.simple_mapper:main",
            "web_server = ros_test.web_server:main",
        ],
    },
)
