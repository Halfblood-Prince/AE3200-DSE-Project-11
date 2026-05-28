#!/usr/bin/env python3
"""Plan through a saved OctoMap .bt file and publish an RViz Path."""

from __future__ import annotations

from os.path import expanduser, expandvars
from pathlib import Path as FilePath
from typing import Iterable

import numpy as np
import rclpy
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from pathfinding.astar import astar
from pathfinding.octomap_grid import OctomapGrid, bt_file_to_numpy_grid


GridIndex = tuple[int, int, int]
WorldPoint = tuple[float, float, float]


def transient_path_qos() -> QoSProfile:
    """Use latched QoS so RViz receives the latest path after it opens."""
    return QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


def parse_xyz(value: Iterable[float], parameter_name: str) -> WorldPoint:
    """Normalize a ROS list parameter into a 3D world coordinate."""
    coords = tuple(float(component) for component in value)
    if len(coords) != 3:
        raise ValueError(f"{parameter_name} must contain exactly three values")
    return coords


def path_to_message(
    octomap_grid: OctomapGrid,
    path: list[GridIndex] | None,
    frame_id: str,
    stamp,
) -> Path:
    """Convert an A* grid-index path into nav_msgs/Path for RViz."""
    msg = Path()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp

    if path is None:
        return msg

    for index in path:
        x, y, z = octomap_grid.grid_to_world(index)
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = stamp
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.w = 1.0
        msg.poses.append(pose)

    return msg


def prepare_planning_grid(octomap_grid: OctomapGrid, start: GridIndex, goal: GridIndex) -> np.ndarray:
    """Copy the occupancy grid and make the requested endpoints traversable."""
    for label, index in (("start", start), ("goal", goal)):
        if not octomap_grid.contains_index(index):
            raise ValueError(f"{label} index {index} is outside the OctoMap grid")

    planning_grid = np.array(octomap_grid.grid, copy=True)
    for x, y, z in (start, goal):
        planning_grid[z, y, x] = 0
    return planning_grid


def plan_path_from_bt(
    bt_path: str | FilePath,
    start_xyz: WorldPoint,
    goal_xyz: WorldPoint,
    *,
    planning_resolution: float | None = None,
    padding: float = 0.5,
    unknown_is_occupied: bool = True,
) -> tuple[OctomapGrid, list[GridIndex] | None, GridIndex, GridIndex]:
    """Build the planning grid from a .bt file and run the bundled A* planner."""
    octomap_grid = bt_file_to_numpy_grid(
        bt_path,
        planning_resolution=planning_resolution,
        padding=padding,
        unknown_is_occupied=unknown_is_occupied,
        include_points=(start_xyz, goal_xyz),
    )
    start_index = octomap_grid.world_to_grid(start_xyz)
    goal_index = octomap_grid.world_to_grid(goal_xyz)
    planning_grid = prepare_planning_grid(octomap_grid, start_index, goal_index)
    path = astar(planning_grid, start_index, goal_index)
    return octomap_grid, path, start_index, goal_index


class AstarPathPublisher(Node):
    """Publish a planned path from a saved OctoMap as nav_msgs/Path."""

    def __init__(self) -> None:
        """Declare planning parameters and create the latched path publisher."""
        super().__init__("astar_path_publisher")

        self.declare_parameter("bt_path", "")
        self.declare_parameter("start_xyz", [0.0, 0.0, 0.5])
        self.declare_parameter("goal_xyz", [3.0, 0.0, 0.5])
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("path_topic", "/astar_path")
        self.declare_parameter("planning_resolution", 0.0)
        self.declare_parameter("padding", 0.5)
        self.declare_parameter("unknown_is_occupied", True)
        self.declare_parameter("publish_period", 1.0)

        path_topic = self.get_parameter("path_topic").value
        publish_period = float(self.get_parameter("publish_period").value)
        self._publisher = self.create_publisher(Path, path_topic, transient_path_qos())
        self._path_message: Path | None = None
        self._planned = False
        self._warned_missing_path = False
        self._timer = self.create_timer(max(0.1, publish_period), self._publish_path)

        self._plan_once()

    def _plan_once(self) -> None:
        """Load the saved map, run A*, and cache the Path message."""
        if self._planned:
            return

        bt_path = expanduser(expandvars(str(self.get_parameter("bt_path").value)))
        if not bt_path:
            if not self._warned_missing_path:
                self.get_logger().warning("Set bt_path to a saved OctoMap .bt file before planning")
                self._warned_missing_path = True
            return

        start_xyz = parse_xyz(self.get_parameter("start_xyz").value, "start_xyz")
        goal_xyz = parse_xyz(self.get_parameter("goal_xyz").value, "goal_xyz")
        planning_resolution_value = float(self.get_parameter("planning_resolution").value)
        planning_resolution = planning_resolution_value if planning_resolution_value > 0.0 else None

        octomap_grid, path, start_index, goal_index = plan_path_from_bt(
            bt_path,
            start_xyz,
            goal_xyz,
            planning_resolution=planning_resolution,
            padding=float(self.get_parameter("padding").value),
            unknown_is_occupied=bool(self.get_parameter("unknown_is_occupied").value),
        )

        frame_id = str(self.get_parameter("frame_id").value)
        self._path_message = path_to_message(
            octomap_grid,
            path,
            frame_id,
            self.get_clock().now().to_msg(),
        )
        self._planned = True

        grid_shape = octomap_grid.grid.shape
        if path is None:
            self.get_logger().warning(
                f"No A* path from {start_index} to {goal_index} in grid "
                f"{grid_shape[2]}x{grid_shape[1]}x{grid_shape[0]}"
            )
        else:
            self.get_logger().info(
                f"Publishing A* path with {len(path)} poses from {bt_path} "
                f"at {octomap_grid.resolution:.3f} m resolution"
            )

    def _publish_path(self) -> None:
        """Publish the cached path, retrying setup while bt_path is missing."""
        if not self._planned:
            self._plan_once()
        if self._path_message is not None:
            self._path_message.header.stamp = self.get_clock().now().to_msg()
            for pose in self._path_message.poses:
                pose.header.stamp = self._path_message.header.stamp
            self._publisher.publish(self._path_message)


def main(args: list[str] | None = None) -> None:
    """Run the saved-map A* path publisher node."""
    rclpy.init(args=args)
    node = AstarPathPublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
