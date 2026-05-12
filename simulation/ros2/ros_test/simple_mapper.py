import math

import rclpy
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan


class SimpleMapper(Node):
    def __init__(self):
        super().__init__("simple_mapper")
        self._resolution = 0.05
        self._width = 600
        self._height = 560
        self._origin_x = -12.0
        self._origin_y = -8.0
        self._lidar_offset_x = 0.45
        self._lidar_offset_y = 0.0
        self._log_odds = [0.0] * (self._width * self._height)
        self._odom = None
        self._got_scan = False

        self.create_subscription(Odometry, "/odom", self._handle_odom, 20)
        self.create_subscription(LaserScan, "/scan", self._handle_scan, 10)
        map_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self._map_pub = self.create_publisher(OccupancyGrid, "/map", map_qos)
        self.create_timer(1.0, self._publish_map)

    def _handle_odom(self, msg):
        self._odom = msg

    def _handle_scan(self, msg):
        if self._odom is None:
            return

        pose = self._odom.pose.pose
        robot_x = pose.position.x
        robot_y = pose.position.y
        robot_yaw = self._yaw_from_quaternion(pose.orientation)
        lidar_x = robot_x + (
            self._lidar_offset_x * math.cos(robot_yaw)
            - self._lidar_offset_y * math.sin(robot_yaw)
        )
        lidar_y = robot_y + (
            self._lidar_offset_x * math.sin(robot_yaw)
            + self._lidar_offset_y * math.cos(robot_yaw)
        )

        angle = msg.angle_min
        for range_value in msg.ranges:
            usable = math.isfinite(range_value)
            clipped_range = min(range_value if usable else msg.range_max, msg.range_max)
            if clipped_range >= msg.range_min:
                end_x = lidar_x + clipped_range * math.cos(robot_yaw + angle)
                end_y = lidar_y + clipped_range * math.sin(robot_yaw + angle)
                self._mark_ray(lidar_x, lidar_y, end_x, end_y, usable and range_value < msg.range_max)
            angle += msg.angle_increment

        if not self._got_scan:
            self.get_logger().info("Building /map from /scan and /odom")
            self._got_scan = True

    def _mark_ray(self, start_x, start_y, end_x, end_y, mark_hit):
        start = self._world_to_grid(start_x, start_y)
        end = self._world_to_grid(end_x, end_y)
        if start is None or end is None:
            return

        cells = self._bresenham(start[0], start[1], end[0], end[1])
        if not cells:
            return

        free_cells = cells[:-4] if mark_hit and len(cells) > 4 else cells
        for x, y in free_cells:
            self._mark_free(x, y)

        if mark_hit:
            self._mark_occupied(cells[-1][0], cells[-1][1])

    def _publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.info.resolution = self._resolution
        msg.info.width = self._width
        msg.info.height = self._height
        msg.info.origin.position.x = self._origin_x
        msg.info.origin.position.y = self._origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = [self._occupancy_value(value) for value in self._log_odds]
        self._map_pub.publish(msg)

    def _world_to_grid(self, x, y):
        grid_x = int((x - self._origin_x) / self._resolution)
        grid_y = int((y - self._origin_y) / self._resolution)
        if 0 <= grid_x < self._width and 0 <= grid_y < self._height:
            return grid_x, grid_y
        return None

    def _add_log_odds(self, x, y, delta):
        index = y * self._width + x
        self._log_odds[index] = max(-4.0, min(4.0, self._log_odds[index] + delta))

    def _mark_free(self, x, y):
        index = y * self._width + x
        if self._log_odds[index] < 0.8:
            self._log_odds[index] = max(-4.0, self._log_odds[index] - 0.08)

    def _mark_occupied(self, center_x, center_y):
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                x = center_x + dx
                y = center_y + dy
                if 0 <= x < self._width and 0 <= y < self._height:
                    self._add_log_odds(x, y, 2.0)

    @staticmethod
    def _occupancy_value(log_odds):
        if log_odds > 0.6:
            return 100
        if log_odds < -0.6:
            return 0
        if -0.2 < log_odds < 0.2:
            return -1
        probability = 1.0 - 1.0 / (1.0 + math.exp(log_odds))
        return int(max(0, min(100, round(probability * 100))))

    @staticmethod
    def _bresenham(x0, y0, x1, y1):
        cells = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        error = dx + dy

        x = x0
        y = y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            twice_error = 2 * error
            if twice_error >= dy:
                error += dy
                x += sx
            if twice_error <= dx:
                error += dx
                y += sy
        return cells

    @staticmethod
    def _yaw_from_quaternion(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleMapper()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
