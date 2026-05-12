#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <optional>
#include <utility>
#include <vector>

namespace
{
class SimpleMapper : public rclcpp::Node
{
  public:
    SimpleMapper() : Node("simple_mapper"), log_odds_(width_ * height_, 0.0)
    {
        odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            20,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                odom_ = std::move(msg);
            });
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                handle_scan(*msg);
            });

        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        map_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("/map", map_qos);
        timer_ = create_wall_timer(std::chrono::seconds(1), [this] {
            publish_map();
        });
    }

  private:
    void handle_scan(const sensor_msgs::msg::LaserScan &msg)
    {
        if (!odom_)
        {
            return;
        }

        const auto &pose = odom_->pose.pose;
        const auto robot_x = pose.position.x;
        const auto robot_y = pose.position.y;
        const auto robot_yaw = yaw_from_quaternion(pose.orientation);
        const auto lidar_x = robot_x + (lidar_offset_x_ * std::cos(robot_yaw) -
                                        lidar_offset_y_ * std::sin(robot_yaw));
        const auto lidar_y = robot_y + (lidar_offset_x_ * std::sin(robot_yaw) +
                                        lidar_offset_y_ * std::cos(robot_yaw));

        auto angle = static_cast<double>(msg.angle_min);
        for (const auto range_value : msg.ranges)
        {
            const auto usable = std::isfinite(range_value);
            const auto clipped_range = std::min(
                usable ? static_cast<double>(range_value) : static_cast<double>(msg.range_max),
                static_cast<double>(msg.range_max));
            if (clipped_range >= msg.range_min)
            {
                const auto end_x = lidar_x + clipped_range * std::cos(robot_yaw + angle);
                const auto end_y = lidar_y + clipped_range * std::sin(robot_yaw + angle);
                mark_ray(lidar_x, lidar_y, end_x, end_y, usable && range_value < msg.range_max);
            }
            angle += msg.angle_increment;
        }

        if (!got_scan_)
        {
            RCLCPP_INFO(get_logger(), "Building /map from /scan and /odom");
            got_scan_ = true;
        }
    }

    void mark_ray(double start_x, double start_y, double end_x, double end_y, bool mark_hit)
    {
        const auto start = world_to_grid(start_x, start_y);
        const auto end = world_to_grid(end_x, end_y);
        if (!start || !end)
        {
            return;
        }

        const auto cells = bresenham(start->first, start->second, end->first, end->second);
        if (cells.empty())
        {
            return;
        }

        const auto free_count = mark_hit && cells.size() > 4 ? cells.size() - 4 : cells.size();
        for (size_t index = 0; index < free_count; ++index)
        {
            mark_free(cells[index].first, cells[index].second);
        }

        if (mark_hit)
        {
            mark_occupied(cells.back().first, cells.back().second);
        }
    }

    void publish_map()
    {
        nav_msgs::msg::OccupancyGrid msg;
        msg.header.stamp = get_clock()->now();
        msg.header.frame_id = "odom";
        msg.info.resolution = resolution_;
        msg.info.width = width_;
        msg.info.height = height_;
        msg.info.origin.position.x = origin_x_;
        msg.info.origin.position.y = origin_y_;
        msg.info.origin.orientation.w = 1.0;
        msg.data.reserve(log_odds_.size());
        for (const auto value : log_odds_)
        {
            msg.data.push_back(occupancy_value(value));
        }
        map_pub_->publish(msg);
    }

    std::optional<std::pair<int, int>> world_to_grid(double x, double y) const
    {
        const auto grid_x = static_cast<int>((x - origin_x_) / resolution_);
        const auto grid_y = static_cast<int>((y - origin_y_) / resolution_);
        if (grid_x >= 0 && grid_x < width_ && grid_y >= 0 && grid_y < height_)
        {
            return std::make_pair(grid_x, grid_y);
        }
        return std::nullopt;
    }

    void add_log_odds(int x, int y, double delta)
    {
        const auto index = static_cast<size_t>(y * width_ + x);
        log_odds_[index] = std::clamp(log_odds_[index] + delta, -4.0, 4.0);
    }

    void mark_free(int x, int y)
    {
        const auto index = static_cast<size_t>(y * width_ + x);
        if (log_odds_[index] < 0.8)
        {
            log_odds_[index] = std::max(-4.0, log_odds_[index] - 0.08);
        }
    }

    void mark_occupied(int center_x, int center_y)
    {
        for (int dx = -1; dx <= 1; ++dx)
        {
            for (int dy = -1; dy <= 1; ++dy)
            {
                const auto x = center_x + dx;
                const auto y = center_y + dy;
                if (x >= 0 && x < width_ && y >= 0 && y < height_)
                {
                    add_log_odds(x, y, 2.0);
                }
            }
        }
    }

    static int8_t occupancy_value(double log_odds)
    {
        if (log_odds > 0.6)
        {
            return 100;
        }
        if (log_odds < -0.6)
        {
            return 0;
        }
        if (log_odds > -0.2 && log_odds < 0.2)
        {
            return -1;
        }
        const auto probability = 1.0 - 1.0 / (1.0 + std::exp(log_odds));
        return static_cast<int8_t>(std::clamp<int>(static_cast<int>(std::lround(probability * 100.0)), 0, 100));
    }

    static std::vector<std::pair<int, int>> bresenham(int x0, int y0, int x1, int y1)
    {
        std::vector<std::pair<int, int>> cells;
        const auto dx = std::abs(x1 - x0);
        const auto dy = -std::abs(y1 - y0);
        const auto sx = x0 < x1 ? 1 : -1;
        const auto sy = y0 < y1 ? 1 : -1;
        auto error = dx + dy;

        auto x = x0;
        auto y = y0;
        while (true)
        {
            cells.emplace_back(x, y);
            if (x == x1 && y == y1)
            {
                break;
            }
            const auto twice_error = 2 * error;
            if (twice_error >= dy)
            {
                error += dy;
                x += sx;
            }
            if (twice_error <= dx)
            {
                error += dx;
                y += sy;
            }
        }
        return cells;
    }

    template <typename QuaternionT>
    static double yaw_from_quaternion(const QuaternionT &q)
    {
        const auto siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
        const auto cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
        return std::atan2(siny_cosp, cosy_cosp);
    }

    const double resolution_{0.05};
    const int width_{600};
    const int height_{560};
    const double origin_x_{-12.0};
    const double origin_y_{-8.0};
    const double lidar_offset_x_{0.45};
    const double lidar_offset_y_{0.0};

    std::vector<double> log_odds_;
    nav_msgs::msg::Odometry::SharedPtr odom_;
    bool got_scan_{false};
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};
} // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SimpleMapper>());
    rclcpp::shutdown();
    return 0;
}
