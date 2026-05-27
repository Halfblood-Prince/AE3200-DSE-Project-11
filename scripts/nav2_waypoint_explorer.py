#!/usr/bin/env python3
from __future__ import annotations

import math
import os
from collections import deque
from enum import Enum
from time import monotonic
from typing import Optional

import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from action_msgs.msg import GoalStatus
    from nav2_msgs.action import NavigateToPose
    from nav2_msgs.srv import SaveMap
except ImportError as error:
    GoalStatus = None
    NavigateToPose = None
    SaveMap = None
    NAV2_IMPORT_ERROR = error
else:
    NAV2_IMPORT_ERROR = None


Cell = tuple[int, int]
Point = tuple[float, float]


def transient_map_qos() -> QoSProfile:
    return QoSProfile(
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        reliability=ReliabilityPolicy.RELIABLE,
    )


class State(Enum):
    EXPLORING = "exploring"
    RETURNING = "returning"
    SETTLING = "settling"
    SAVING = "saving"
    COMPLETE = "complete"


class Nav2WaypointExplorer(Node):
    def __init__(self) -> None:
        super().__init__("nav2_waypoint_explorer")

        self.declare_parameter("map_topic", "/map_valid")
        self.declare_parameter("map_save_path", "maps/complete_environment")
        self.declare_parameter("min_exploration_goals", 10)
        self.declare_parameter("frontier_timeout_sec", 45.0)
        self.declare_parameter("initial_scan_sec", 10.0)
        self.declare_parameter("loop_closure_settle_sec", 8.0)
        self.declare_parameter("frontier_sample_step_m", 0.35)
        self.declare_parameter("frontier_clearance_m", 0.45)
        self.declare_parameter("frontier_min_distance_m", 3.0)
        self.declare_parameter("frontier_max_distance_m", 18.0)
        self.declare_parameter("frontier_unknown_radius_m", 0.9)
        self.declare_parameter("frontier_min_unknown_cells", 6)
        self.declare_parameter("return_to_start", True)

        self._client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self._save_map = self.create_client(SaveMap, "map_saver/save_map")
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._map: Optional[OccupancyGrid] = None
        self._state = State.EXPLORING
        self._active = False
        self._goal_count = 0
        self._current_goal: Optional[Point] = None
        self._failed_goals: list[Point] = []
        self._visited_goals: list[Point] = []
        self._start_pose: Optional[Point] = None
        self._start_time = monotonic()
        self._last_frontier_time = monotonic()
        self._settle_started_at: Optional[float] = None
        self._save_requested = False
        self._safe_cell_cache_enabled = False
        self._safe_cell_cache: dict[Cell, bool] = {}

        map_topic = self.get_parameter("map_topic").value
        self._map_sub = self.create_subscription(
            OccupancyGrid,
            map_topic,
            self._handle_map,
            transient_map_qos(),
        )
        self._timer = self.create_timer(2.0, self._tick)

    def _handle_map(self, msg: OccupancyGrid) -> None:
        self._map = msg

    def _parameter_double(self, name: str) -> float:
        return float(self.get_parameter(name).value)

    def _parameter_int(self, name: str) -> int:
        return int(self.get_parameter(name).value)

    def _tick(self) -> None:
        if self._state == State.COMPLETE or self._active:
            return
        if self._map is None:
            map_topic = self.get_parameter("map_topic").value
            self.get_logger().info(
                f"Waiting for {map_topic} before autonomous frontier exploration"
            )
            return
        if not self._client.wait_for_server(timeout_sec=0.1):
            self.get_logger().info("Waiting for Nav2 navigate_to_pose action server")
            return

        robot = self._robot_pose()
        if robot is None:
            return
        if self._start_pose is None:
            self._start_pose = robot
            self.get_logger().info(
                f"Recorded SLAM start pose at x={robot[0]:.2f}, y={robot[1]:.2f}"
            )

        if self._state == State.EXPLORING:
            self._continue_exploration(robot)
        elif self._state == State.RETURNING:
            self._send_return_goal(robot)
        elif self._state == State.SETTLING:
            self._wait_then_save()
        elif self._state == State.SAVING:
            self._request_map_save()

    def _continue_exploration(self, robot: Point) -> None:
        known_free = self._known_free_cell_count()
        initial_scan_sec = self._parameter_double("initial_scan_sec")
        elapsed_start = monotonic() - self._start_time
        if self._goal_count == 0 and elapsed_start < initial_scan_sec:
            self._publish_initial_scan_turn()
            self.get_logger().info(
                "Building the initial SLAM bubble before frontier navigation "
                f"({known_free} free cells)"
            )
            return

        target = self._choose_frontier_goal(robot)
        if target is not None:
            self._publish_stop()
            self._last_frontier_time = monotonic()
            self._send_goal(target, robot, "frontier")
            return

        elapsed = monotonic() - self._last_frontier_time
        min_goals = self._parameter_int("min_exploration_goals")
        timeout = self._parameter_double("frontier_timeout_sec")
        if self._goal_count < min_goals or elapsed < timeout:
            if self._goal_count == 0 and known_free < 250:
                self._publish_initial_scan_turn()
            self.get_logger().info(
                "No reachable frontier goal yet; waiting for more known free space "
                f"({known_free} free cells, {self._goal_count}/{min_goals} goals, "
                f"{elapsed:.0f}/{timeout:.0f}s quiet)"
            )
            return

        self._publish_stop()
        if bool(self.get_parameter("return_to_start").value):
            self.get_logger().info(
                "No frontiers remain; returning near the start pose so the mapper can settle"
            )
            self._state = State.RETURNING
        else:
            self.get_logger().info("No frontiers remain; saving the completed map")
            self._state = State.SAVING

    def _send_return_goal(self, robot: Point) -> None:
        if self._start_pose is None:
            self._state = State.SAVING
            return
        if math.hypot(robot[0] - self._start_pose[0], robot[1] - self._start_pose[1]) < 0.75:
            self.get_logger().info(
                "Robot is back near the start pose; waiting for SLAM to settle"
            )
            self._state = State.SETTLING
            self._settle_started_at = monotonic()
            return
        self._send_goal(self._start_pose, robot, "loop-closure return")

    def _wait_then_save(self) -> None:
        settle = self._parameter_double("loop_closure_settle_sec")
        if self._settle_started_at is None:
            self._settle_started_at = monotonic()
        if monotonic() - self._settle_started_at < settle:
            return

        self.get_logger().info(
            "SLAM settle period complete; saving map and leaving Nav2 ready"
        )
        self._state = State.SAVING

    def _request_map_save(self) -> None:
        if self._save_requested:
            return
        if not self._save_map.wait_for_service(timeout_sec=0.1):
            self.get_logger().info("Waiting for map_saver/save_map service")
            return

        map_url = self.get_parameter("map_save_path").value
        parent = os.path.dirname(map_url)
        if parent:
            os.makedirs(parent, exist_ok=True)

        request = SaveMap.Request()
        request.map_topic = self.get_parameter("map_topic").value
        request.map_url = map_url
        request.image_format = "pgm"
        request.map_mode = "trinary"
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65

        self._save_requested = True
        future = self._save_map.call_async(request)
        future.add_done_callback(self._map_saved)

    def _map_saved(self, future) -> None:
        try:
            response = future.result()
            if response.result:
                path = self.get_parameter("map_save_path").value
                self.get_logger().info(
                    f"Saved completed map to {path}.yaml/.pgm. Exploration is stopped; "
                    "Nav2 remains active for pathfinding goals."
                )
                self._state = State.COMPLETE
            else:
                self.get_logger().error("map_saver reported failure; will retry")
                self._save_requested = False
        except Exception as error:
            self.get_logger().error(f"Map save failed: {error}")
            self._save_requested = False

    def _robot_pose(self) -> Optional[Point]:
        try:
            transform = self._tf_buffer.lookup_transform("map", "base_link", Time())
            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
            )
        except TransformException as error:
            self.get_logger().info(f"Waiting for map -> base_link TF: {error}")
            return None

    def _known_free_cell_count(self) -> int:
        if self._map is None:
            return 0
        return sum(1 for value in self._map.data if 0 <= value < 50)

    def _publish_initial_scan_turn(self) -> None:
        cmd = Twist()
        cmd.angular.z = 0.28
        self._cmd_pub.publish(cmd)

    def _publish_stop(self) -> None:
        self._cmd_pub.publish(Twist())

    def _choose_frontier_goal(self, robot: Point) -> Optional[Point]:
        if self._map is None:
            return None

        robot_cell = self._world_to_cell(robot[0], robot[1])
        if robot_cell is None:
            return None

        self._safe_cell_cache.clear()
        self._safe_cell_cache_enabled = True
        reachable = self._reachable_safe_cells(robot_cell)
        self._safe_cell_cache_enabled = False
        self._safe_cell_cache.clear()

        if not reachable:
            self.get_logger().warning("No reachable known-free cells found around the robot yet")
            return None

        resolution = self._map.info.resolution
        step = max(1, int(self._parameter_double("frontier_sample_step_m") / resolution))
        min_distance = self._parameter_double("frontier_min_distance_m")
        max_distance = self._parameter_double("frontier_max_distance_m")
        unknown_radius = self._parameter_double("frontier_unknown_radius_m")
        min_unknown = self._parameter_int("frontier_min_unknown_cells")

        best: Optional[Point] = None
        best_score = -math.inf
        sampled = 0
        near_unknown = 0

        for mx, my in reachable:
            if mx % step or my % step:
                continue
            sampled += 1

            world = self._cell_to_world(mx, my)
            robot_distance = math.hypot(world[0] - robot[0], world[1] - robot[1])
            if robot_distance < min_distance or robot_distance > max_distance:
                continue
            if self._recently_seen(world[0], world[1]):
                continue

            unknown = self._unknown_neighbor_count(mx, my, unknown_radius)
            if unknown < min_unknown:
                continue
            near_unknown += 1

            start_bonus = 0.0
            if self._start_pose is not None:
                start_bonus = 0.1 * math.hypot(
                    world[0] - self._start_pose[0],
                    world[1] - self._start_pose[1],
                )
            distance_bonus = min(robot_distance, max_distance) * 0.8
            score = unknown + distance_bonus + start_bonus
            if score > best_score:
                best = world
                best_score = score

        if best is None:
            self.get_logger().info(
                "Frontier scan found no target "
                f"({len(reachable)} reachable cells, {sampled} sampled, "
                f"{near_unknown} near unknown)"
            )
        return best

    def _cell_to_world(self, mx: int, my: int) -> Point:
        assert self._map is not None
        origin = self._map.info.origin.position
        return (
            origin.x + (mx + 0.5) * self._map.info.resolution,
            origin.y + (my + 0.5) * self._map.info.resolution,
        )

    def _world_to_cell(self, x: float, y: float) -> Optional[Cell]:
        assert self._map is not None
        origin = self._map.info.origin.position
        mx = int((x - origin.x) / self._map.info.resolution)
        my = int((y - origin.y) / self._map.info.resolution)
        if 0 <= mx < self._map.info.width and 0 <= my < self._map.info.height:
            return mx, my
        return None

    def _cell_value(self, mx: int, my: int) -> int:
        assert self._map is not None
        return int(self._map.data[my * self._map.info.width + mx])

    def _is_safe_free_cell(self, mx: int, my: int) -> bool:
        assert self._map is not None
        if mx < 0 or my < 0 or mx >= self._map.info.width or my >= self._map.info.height:
            return False

        key = (mx, my)
        if self._safe_cell_cache_enabled and key in self._safe_cell_cache:
            return self._safe_cell_cache[key]

        center_value = self._cell_value(mx, my)
        if center_value < 0 or center_value >= 50:
            return self._cache_safe_cell(key, False)

        clearance = self._parameter_double("frontier_clearance_m")
        clearance_cells = max(2, int(clearance / self._map.info.resolution))
        safe = True
        for dy in range(-clearance_cells, clearance_cells + 1):
            if not safe:
                break
            for dx in range(-clearance_cells, clearance_cells + 1):
                if dx * dx + dy * dy > clearance_cells * clearance_cells:
                    continue
                x = mx + dx
                y = my + dy
                if (
                    x < 0
                    or y < 0
                    or x >= self._map.info.width
                    or y >= self._map.info.height
                ):
                    safe = False
                    break
                if self._cell_value(x, y) >= 50:
                    safe = False
                    break

        return self._cache_safe_cell(key, safe)

    def _cache_safe_cell(self, key: Cell, value: bool) -> bool:
        if self._safe_cell_cache_enabled:
            self._safe_cell_cache[key] = value
        return value

    def _reachable_safe_cells(self, start: Cell) -> list[Cell]:
        if not self._is_safe_free_cell(start[0], start[1]):
            nearby = self._nearest_safe_cell(start)
            if nearby is None:
                return []
            start = nearby

        queue = deque([start])
        visited = {start}
        cells: list[Cell] = []

        while queue:
            mx, my = queue.popleft()
            cells.append((mx, my))

            for neighbor in ((mx + 1, my), (mx - 1, my), (mx, my + 1), (mx, my - 1)):
                if neighbor in visited:
                    continue
                if not self._is_safe_free_cell(neighbor[0], neighbor[1]):
                    continue
                visited.add(neighbor)
                queue.append(neighbor)

        return cells

    def _nearest_safe_cell(self, start: Cell) -> Optional[Cell]:
        assert self._map is not None
        max_radius = max(2, int(1.0 / self._map.info.resolution))
        for radius in range(1, max_radius + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    mx = start[0] + dx
                    my = start[1] + dy
                    if (
                        0 <= mx < self._map.info.width
                        and 0 <= my < self._map.info.height
                        and self._is_safe_free_cell(mx, my)
                    ):
                        return mx, my
        return None

    def _unknown_neighbor_count(self, mx: int, my: int, radius_m: float) -> int:
        assert self._map is not None
        radius = max(2, int(radius_m / self._map.info.resolution))
        count = 0
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                x = mx + dx
                y = my + dy
                if x < 0 or y < 0 or x >= self._map.info.width or y >= self._map.info.height:
                    continue
                if self._cell_value(x, y) < 0:
                    count += 1
        return count

    def _recently_seen(self, x: float, y: float) -> bool:
        def close_to(point: Point) -> bool:
            return math.hypot(x - point[0], y - point[1]) < 0.9

        for point in self._failed_goals[-30:]:
            if close_to(point):
                return True

        for point in self._visited_goals[-50:]:
            if close_to(point):
                return True

        return False

    def _send_goal(self, target: Point, robot: Point, label: str) -> None:
        yaw = math.atan2(target[1] - robot[1], target[0] - robot[0])
        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(target[0], target[1], yaw)

        self._goal_count += 1
        self.get_logger().info(
            f"Sending Nav2 {label} goal {self._goal_count}: "
            f"x={target[0]:.2f}, y={target[1]:.2f}, yaw={yaw:.2f}"
        )

        self._active = True
        self._current_goal = target
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future) -> None:
        try:
            goal_handle = future.result()
        except Exception as error:
            self.get_logger().warning(f"Nav2 goal request failed: {error}")
            self._mark_current_goal_failed()
            self._active = False
            return

        if not goal_handle.accepted:
            self.get_logger().warning("Nav2 goal rejected; choosing a new goal")
            self._mark_current_goal_failed()
            self._active = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future) -> None:
        try:
            result = future.result()
            status = result.status
        except Exception as error:
            self.get_logger().warning(f"Nav2 goal result failed: {error}")
            status = GoalStatus.STATUS_UNKNOWN

        self.get_logger().info(f"Nav2 goal finished with status {status}")
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self._current_goal is not None:
                self._visited_goals.append(self._current_goal)
        else:
            self._mark_current_goal_failed()

        if self._state == State.RETURNING:
            self._state = State.SETTLING
            self._settle_started_at = monotonic()

        self._current_goal = None
        self._active = False

    def _mark_current_goal_failed(self) -> None:
        if self._current_goal is not None:
            self._failed_goals.append(self._current_goal)
            self._current_goal = None

    def _make_pose(self, x: float, y: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose


def main(args: list[str] | None = None) -> None:
    rclpy.init(args=args)
    if NAV2_IMPORT_ERROR is not None:
        node = Node("nav2_waypoint_explorer")
        try:
            node.get_logger().error(
                "nav2_waypoint_explorer requires Nav2 Python message bindings. "
                f"Install the Lyrical Nav2 packages that provide nav2_msgs: {NAV2_IMPORT_ERROR}"
            )
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return

    node = Nav2WaypointExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
