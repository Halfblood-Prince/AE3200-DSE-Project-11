#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <stdexcept>

namespace
{
class AutoDrive : public rclcpp::Node
{
  public:
    AutoDrive() : Node("auto_drive")
    {
        cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
        points_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
            "/points_raw",
            10,
            [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
                cloud_ = std::move(msg);
                if (!logged_first_cloud_)
                {
                    RCLCPP_INFO(get_logger(), "Auto drive received /points_raw and is publishing /cmd_vel");
                    logged_first_cloud_ = true;
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
        if (!cloud_)
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

        try
        {
            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*cloud_, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*cloud_, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*cloud_, "z");
            for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
            {
                const auto x = static_cast<double>(*iter_x);
                const auto y = static_cast<double>(*iter_y);
                const auto z = static_cast<double>(*iter_z);
                if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) || x <= 0.0)
                {
                    continue;
                }
                if (z < -0.35 || z > 1.2)
                {
                    continue;
                }

                const auto angle = std::atan2(y, x);
                if (angle >= start_angle && angle <= end_angle)
                {
                    best = std::min(best, std::hypot(x, y));
                }
            }
        }
        catch (const std::runtime_error &error)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                5000,
                "Unable to read XYZ fields from /points_raw: %s",
                error.what());
        }
        return best;
    }

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr points_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    sensor_msgs::msg::PointCloud2::SharedPtr cloud_;
    bool logged_first_cloud_{false};
};
} // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AutoDrive>());
    rclcpp::shutdown();
    return 0;
}
