import math
import os
import time
from collections import deque

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import SaveMap
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class Nav2WaypointExplorer(Node):
    def __init__(self):
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

        self._map = None
        self._state = "exploring"
        self._active = False
        self._goal_count = 0
        self._current_goal = None
        self._failed_goals = []
        self._visited_goals = []
        self._start_pose = None
        self._start_time = time.monotonic()
        self._last_frontier_time = time.monotonic()
        self._settle_started_at = None
        self._save_requested = False

        map_topic = self.get_parameter("map_topic").value
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(OccupancyGrid, map_topic, self._handle_map, map_qos)
        self.create_timer(2.0, self._tick)

    def _handle_map(self, msg):
        self._map = msg

    def _tick(self):
        if self._state == "complete" or self._active:
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

        if self._state == "exploring":
            self._continue_exploration(robot)
        elif self._state == "returning":
            self._send_return_goal(robot)
        elif self._state == "settling":
            self._wait_then_save()
        elif self._state == "saving":
            self._request_map_save()

    def _continue_exploration(self, robot):
        known_free = self._known_free_cell_count()
        initial_scan_sec = self.get_parameter("initial_scan_sec").value
        if self._goal_count == 0 and time.monotonic() - self._start_time < initial_scan_sec:
            self._publish_initial_scan_turn()
            self.get_logger().info(
                f"Building the initial SLAM bubble before frontier navigation ({known_free} free cells)"
            )
            return

        target = self._choose_frontier_goal(robot)
        if target is not None:
            self._publish_stop()
            self._last_frontier_time = time.monotonic()
            self._send_goal(target, robot, "frontier")
            return

        elapsed = time.monotonic() - self._last_frontier_time
        min_goals = self.get_parameter("min_exploration_goals").value
        timeout = self.get_parameter("frontier_timeout_sec").value
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
        if self.get_parameter("return_to_start").value:
            self.get_logger().info(
                "No frontiers remain; returning near the start pose so slam_toolbox can close the loop"
            )
            self._state = "returning"
        else:
            self.get_logger().info("No frontiers remain; saving the completed map")
            self._state = "saving"

    def _send_return_goal(self, robot):
        if self._start_pose is None:
            self._state = "saving"
            return
        if math.hypot(robot[0] - self._start_pose[0], robot[1] - self._start_pose[1]) < 0.75:
            self.get_logger().info("Robot is back near the start pose; waiting for SLAM to settle")
            self._state = "settling"
            self._settle_started_at = time.monotonic()
            return
        self._send_goal(self._start_pose, robot, "loop-closure return")

    def _wait_then_save(self):
        settle = self.get_parameter("loop_closure_settle_sec").value
        if self._settle_started_at is None:
            self._settle_started_at = time.monotonic()
        if time.monotonic() - self._settle_started_at < settle:
            return
        self.get_logger().info("SLAM settle period complete; saving map and leaving Nav2 ready")
        self._state = "saving"

    def _request_map_save(self):
        if self._save_requested:
            return
        if not self._save_map.wait_for_service(timeout_sec=0.1):
            self.get_logger().info("Waiting for map_saver/save_map service")
            return

        map_url = self.get_parameter("map_save_path").value
        directory = os.path.dirname(map_url)
        if directory:
            os.makedirs(directory, exist_ok=True)

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

    def _map_saved(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self.get_logger().error(f"Map save failed: {exc}")
            self._save_requested = False
            return

        if response.result:
            path = self.get_parameter("map_save_path").value
            self.get_logger().info(
                f"Saved completed map to {path}.yaml/.pgm. "
                "Exploration is stopped; Nav2 remains active for pathfinding goals."
            )
            self._state = "complete"
        else:
            self.get_logger().error("map_saver reported failure; will retry")
            self._save_requested = False

    def _robot_pose(self):
        try:
            transform = self._tf_buffer.lookup_transform("map", "base_link", Time())
        except TransformException as exc:
            self.get_logger().info(f"Waiting for map -> base_link TF: {exc}")
            return None

        return (
            transform.transform.translation.x,
            transform.transform.translation.y,
        )

    def _known_free_cell_count(self):
        if self._map is None:
            return 0
        return sum(1 for value in self._map.data if 0 <= value < 50)

    def _publish_initial_scan_turn(self):
        cmd = Twist()
        cmd.angular.z = 0.28
        self._cmd_pub.publish(cmd)

    def _publish_stop(self):
        self._cmd_pub.publish(Twist())

    def _choose_frontier_goal(self, robot):
        grid = self._map
        resolution = grid.info.resolution
        robot_cell = self._world_to_cell(robot[0], robot[1])
        if robot_cell is None:
            return None

        self._safe_cell_cache = {}
        reachable = self._reachable_safe_cells(robot_cell)
        self._safe_cell_cache = None
        if not reachable:
            self.get_logger().warn("No reachable known-free cells found around the robot yet")
            return None

        step = max(1, int(self.get_parameter("frontier_sample_step_m").value / resolution))
        min_distance = self.get_parameter("frontier_min_distance_m").value
        max_distance = self.get_parameter("frontier_max_distance_m").value
        unknown_radius = self.get_parameter("frontier_unknown_radius_m").value
        min_unknown = self.get_parameter("frontier_min_unknown_cells").value
        best = None
        best_score = -1.0
        sampled = 0
        near_unknown = 0

        for mx, my in reachable:
            if mx % step or my % step:
                continue
            sampled += 1

            x, y = self._cell_to_world(mx, my)
            robot_distance = math.hypot(x - robot[0], y - robot[1])
            if robot_distance < min_distance or robot_distance > max_distance:
                continue
            if self._recently_seen(x, y):
                continue

            unknown = self._unknown_neighbor_count(mx, my, unknown_radius)
            if unknown < min_unknown:
                continue
            near_unknown += 1

            start_bonus = 0.0
            if self._start_pose is not None:
                start_bonus = 0.1 * math.hypot(x - self._start_pose[0], y - self._start_pose[1])
            distance_bonus = min(robot_distance, max_distance) * 0.8
            score = unknown + distance_bonus + start_bonus
            if score > best_score:
                best = (x, y)
                best_score = score

        if best is None:
            self.get_logger().info(
                "Frontier scan found no target "
                f"({len(reachable)} reachable cells, {sampled} sampled, "
                f"{near_unknown} near unknown)"
            )
        return best

    def _cell_to_world(self, mx, my):
        grid = self._map
        origin = grid.info.origin.position
        return (
            origin.x + (mx + 0.5) * grid.info.resolution,
            origin.y + (my + 0.5) * grid.info.resolution,
        )

    def _world_to_cell(self, x, y):
        grid = self._map
        origin = grid.info.origin.position
        mx = int((x - origin.x) / grid.info.resolution)
        my = int((y - origin.y) / grid.info.resolution)
        if 0 <= mx < grid.info.width and 0 <= my < grid.info.height:
            return mx, my
        return None

    def _cell_value(self, mx, my):
        return self._map.data[my * self._map.info.width + mx]

    def _is_safe_free_cell(self, mx, my):
        cache = getattr(self, "_safe_cell_cache", None)
        if cache is not None and (mx, my) in cache:
            return cache[(mx, my)]

        grid = self._map
        center_value = self._cell_value(mx, my)
        if center_value < 0 or center_value >= 50:
            if cache is not None:
                cache[(mx, my)] = False
            return False

        clearance = self.get_parameter("frontier_clearance_m").value
        clearance_cells = max(2, int(clearance / grid.info.resolution))
        safe = True
        for dy in range(-clearance_cells, clearance_cells + 1):
            for dx in range(-clearance_cells, clearance_cells + 1):
                if dx * dx + dy * dy > clearance_cells * clearance_cells:
                    continue
                x = mx + dx
                y = my + dy
                if x < 0 or y < 0 or x >= grid.info.width or y >= grid.info.height:
                    safe = False
                    break
                value = self._cell_value(x, y)
                if value >= 50:
                    safe = False
                    break
            if not safe:
                break

        if cache is not None:
            cache[(mx, my)] = safe
        return safe

    def _reachable_safe_cells(self, start):
        if not self._is_safe_free_cell(start[0], start[1]):
            nearby = self._nearest_safe_cell(start)
            if nearby is None:
                return set()
            start = nearby

        queue = deque([start])
        visited = {start}
        while queue:
            mx, my = queue.popleft()
            for nx, ny in ((mx + 1, my), (mx - 1, my), (mx, my + 1), (mx, my - 1)):
                if (nx, ny) in visited:
                    continue
                if not self._is_safe_free_cell(nx, ny):
                    continue
                visited.add((nx, ny))
                queue.append((nx, ny))
        return visited

    def _nearest_safe_cell(self, start):
        grid = self._map
        max_radius = max(2, int(1.0 / grid.info.resolution))
        for radius in range(1, max_radius + 1):
            for dy in range(-radius, radius + 1):
                for dx in range(-radius, radius + 1):
                    if abs(dx) != radius and abs(dy) != radius:
                        continue
                    mx = start[0] + dx
                    my = start[1] + dy
                    if 0 <= mx < grid.info.width and 0 <= my < grid.info.height:
                        if self._is_safe_free_cell(mx, my):
                            return mx, my
        return None

    def _unknown_neighbor_count(self, mx, my, radius_m):
        grid = self._map
        radius = max(2, int(radius_m / grid.info.resolution))
        count = 0
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                x = mx + dx
                y = my + dy
                if x < 0 or y < 0 or x >= grid.info.width or y >= grid.info.height:
                    continue
                if self._cell_value(x, y) < 0:
                    count += 1
        return count

    def _recently_seen(self, x, y):
        recent = self._failed_goals[-30:] + self._visited_goals[-50:]
        return any(math.hypot(x - px, y - py) < 0.9 for px, py in recent)

    def _send_goal(self, target, robot, label):
        x, y = target
        yaw = math.atan2(y - robot[1], x - robot[0])
        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose(x, y, yaw)
        self._goal_count += 1
        self.get_logger().info(
            f"Sending Nav2 {label} goal {self._goal_count}: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}"
        )
        self._active = True
        self._current_goal = (x, y)
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Nav2 goal rejected; choosing a new goal")
            if self._current_goal is not None:
                self._failed_goals.append(self._current_goal)
            self._current_goal = None
            self._active = False
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result()
        status = result.status
        self.get_logger().info(f"Nav2 goal finished with status {status}")
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self._current_goal is not None:
                self._visited_goals.append(self._current_goal)
        elif self._current_goal is not None:
            self._failed_goals.append(self._current_goal)

        if self._state == "returning":
            self._state = "settling"
            self._settle_started_at = time.monotonic()
        self._current_goal = None
        self._active = False

    def _make_pose(self, x, y, yaw):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = Nav2WaypointExplorer()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
