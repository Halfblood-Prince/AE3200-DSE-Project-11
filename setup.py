"""Install the ROS 2 Python nodes and shared Gazebo/RViz assets."""

from glob import glob
from setuptools import find_packages, setup


package_name = "ros_test"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test", "tests"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/config", glob("config/*")),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/robot", glob("robot/*")),
        (f"share/{package_name}/rviz", glob("rviz/*")),
    ],
    install_requires=["setuptools", "numpy"],
    extras_require={"test": ["pytest", "pytest-cov"]},
    zip_safe=True,
    maintainer="user",
    maintainer_email="user@example.com",
    description="Gazebo Harmonic 3D lidar mapping demo for ROS 2 Jazzy.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "cloud_filter = ros_test.cloud_filter:main",
            "astar_path_publisher = ros_test.astar_path_publisher:main",
            "map_filter = ros_test.map_filter:main",
            "map_monitor = ros_test.map_monitor:main",
            "odom_to_tf = ros_test.odom_to_tf:main",
        ],
    },
)
