#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <memory>
#include <string>

namespace
{
class ScanToChassis : public rclcpp::Node
{
  public:
    ScanToChassis() : Node("scan_to_chassis")
    {
        declare_parameter<std::string>("scan_frame_id", "lidar_link");
        publisher_ = create_publisher<sensor_msgs::msg::LaserScan>("/scan", 10);
        subscription_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan_raw",
            10,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                handle_scan(std::move(msg));
            });
    }

  private:
    void handle_scan(sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        msg->header.frame_id = get_parameter("scan_frame_id").as_string();
        publisher_->publish(*msg);

        if (!logged_first_scan_)
        {
            RCLCPP_INFO(
                get_logger(),
                "Republishing /scan_raw as /scan with frame_id '%s'",
                msg->header.frame_id.c_str());
            logged_first_scan_ = true;
        }
    }

    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    bool logged_first_scan_{false};
};
} // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ScanToChassis>());
    rclcpp::shutdown();
    return 0;
}
