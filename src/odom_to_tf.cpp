#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>

namespace
{
class OdomToTf : public rclcpp::Node
{
  public:
    OdomToTf() : Node("odom_to_tf"), broadcaster_(std::make_unique<tf2_ros::TransformBroadcaster>(*this))
    {
        subscription_ = create_subscription<nav_msgs::msg::Odometry>(
            "/odom",
            10,
            [this](nav_msgs::msg::Odometry::SharedPtr msg) {
                handle_odom(*msg);
            });
    }

  private:
    void handle_odom(const nav_msgs::msg::Odometry &msg)
    {
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = msg.header.stamp;
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_link";
        transform.transform.translation.x = msg.pose.pose.position.x;
        transform.transform.translation.y = msg.pose.pose.position.y;
        transform.transform.translation.z = msg.pose.pose.position.z;
        transform.transform.rotation = msg.pose.pose.orientation;
        broadcaster_->sendTransform(transform);

        if (!logged_first_odom_)
        {
            RCLCPP_INFO(
                get_logger(),
                "Publishing TF odom -> base_link from /odom (source frames: '%s' -> '%s')",
                msg.header.frame_id.c_str(),
                msg.child_frame_id.c_str());
            logged_first_odom_ = true;
        }
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    bool logged_first_odom_{false};
};
} // namespace

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdomToTf>());
    rclcpp::shutdown();
    return 0;
}
