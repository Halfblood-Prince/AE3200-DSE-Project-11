#include <nav_msgs/msg/occupancy_grid.hpp>
#include <rclcpp/rclcpp.hpp>

#include <memory>
#include <string>

namespace
{
class MapFilter : public rclcpp::Node
{
  public:
    MapFilter() : Node("map_filter")
    {
        declare_parameter<std::string>("input_topic", "/map");
        declare_parameter<std::string>("output_topic", "/map_valid");

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        const auto input_topic = get_parameter("input_topic").as_string();
        const auto output_topic = get_parameter("output_topic").as_string();

        publisher_ = create_publisher<nav_msgs::msg::OccupancyGrid>(output_topic, qos);
        subscription_ = create_subscription<nav_msgs::msg::OccupancyGrid>(
            input_topic,
            qos,
            [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                handle_map(*msg);
            });
    }

  private:
    void handle_map(const nav_msgs::msg::OccupancyGrid &msg)
    {
        if (msg.info.width == 0 || msg.info.height == 0)
        {
            ++dropped_empty_;
            if (dropped_empty_ == 1)
            {
                RCLCPP_WARN(get_logger(), "Dropping empty /map from the mapper");
            }
            return;
        }

        publisher_->publish(msg);
        if (!published_first_)
        {
            RCLCPP_INFO(
                get_logger(),
                "Publishing filtered map on /map_valid (%ux%u, resolution %.3f)",
                msg.info.width,
                msg.info.height,
                msg.info.resolution);
            published_first_ = true;
        }
    }

    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr publisher_;
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr subscription_;
    int dropped_empty_{0};
    bool published_first_{false};
};
} // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MapFilter>());
    rclcpp::shutdown();
    return 0;
}
