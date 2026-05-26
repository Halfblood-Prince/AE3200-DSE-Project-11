#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace
{
class MapMonitor : public rclcpp::Node
{
  public:
    MapMonitor() : Node("map_monitor")
    {
        declare_parameter<std::string>("map_topic", "/map_valid");
        const auto map_topic = get_parameter("map_topic").as_string();
        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();

        subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic,
            qos,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                handle_map(*msg);
            });
        timer_ = create_wall_timer(std::chrono::seconds(5), [this] {
            report_status();
        });
    }

  private:
    void handle_map(const nav_msgs::msg::OccupancyGrid &msg)
    {
        if (received_map_)
        {
            return;
        }
        if (msg.info.width == 0 || msg.info.height == 0)
        {
            RCLCPP_WARN(get_logger(), "Ignoring empty map while waiting for lidar returns");
            return;
        }

        received_map_ = true;
        const auto map_topic = get_parameter("map_topic").as_string();
        RCLCPP_INFO(
            get_logger(),
            "Received %s (%ux%u, resolution %.3f)",
            map_topic.c_str(),
            msg.info.width,
            msg.info.height,
            msg.info.resolution);
    }

    void report_status()
    {
        if (received_map_)
        {
            return;
        }
        seconds_waited_ += 5;
        RCLCPP_WARN(
            get_logger(),
            "Still waiting for filtered map after %ds. Move the robot and confirm /points_raw "
            "uses frame lidar_link with map -> odom -> base_link -> lidar_link TF.",
            seconds_waited_);
    }

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool received_map_{false};
    int seconds_waited_{0};
};
} // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapMonitor>());
    rclcpp::shutdown();
    return 0;
}
