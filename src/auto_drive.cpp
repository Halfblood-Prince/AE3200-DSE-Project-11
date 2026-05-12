#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>

namespace
{
class AutoDrive : public rclcpp::Node
{
  public:
    AutoDrive() : Node("auto_drive")
    {
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan",
            10,
            [this](sensor_msgs::msg::LaserScan::SharedPtr msg) {
                scan_ = std::move(msg);
                if (!logged_first_scan_)
                {
                    RCLCPP_INFO(get_logger(), "Auto drive received /scan and is publishing /cmd_vel");
                    logged_first_scan_ = true;
                }
            });
        timer_ = create_wall_timer(std::chrono::milliseconds(100), [this] {
            publish_cmd();
        });
    }

  private:
    void publish_cmd()
    {
        geometry_msgs::msg::Twist cmd;
        if (!scan_)
        {
            cmd_pub_->publish(cmd);
            return;
        }

        const auto front = sector_min(-0.35, 0.35);
        const auto left = sector_min(0.35, 1.2);
        const auto right = sector_min(-1.2, -0.35);

        if (front < 0.75)
        {
            cmd.linear.x = 0.0;
            cmd.angular.z = left < right ? -0.8 : 0.8;
        }
        else
        {
            cmd.linear.x = 0.25;
            cmd.angular.z = 0.18;
        }

        cmd_pub_->publish(cmd);
    }

    double sector_min(double start_angle, double end_angle) const
    {
        auto best = std::numeric_limits<double>::infinity();
        auto angle = static_cast<double>(scan_->angle_min);
        for (const auto value : scan_->ranges)
        {
            if (angle >= start_angle && angle <= end_angle && std::isfinite(value))
            {
                best = std::min(best, static_cast<double>(value));
            }
            angle += scan_->angle_increment;
        }
        return best;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::LaserScan::SharedPtr scan_;
    bool logged_first_scan_{false};
};
} // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoDrive>());
    rclcpp::shutdown();
    return 0;
}
