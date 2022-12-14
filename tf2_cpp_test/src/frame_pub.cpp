#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "tf2/exceptions.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

using namespace std::chrono_literals;

class FramePub : public rclcpp::Node
{
public:
    FramePub(): Node("frame_pub"), cnt(0)
    {
        timer_ = this->create_wall_timer(10ms, std::bind(&FramePub::TimerCallback, this));
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

        StaticTransform("base", "arm", 1, 0, 0, 0, 0, 0);
        StaticTransform("wrist", "tip", 0, 0, 1, 0, 0, 0);
    }

private:
    int cnt;
    rclcpp::TimerBase::SharedPtr timer_;
    void TimerCallback()
    {
        cnt += 1;
        Transform("world", "base", 0, 0, 0, 0, tf2Radians(cnt), 0);
        Transform("arm", "wrist", 0, 0, 0, tf2Radians(cnt*10), 0, 0);
    }

    void Transform(const std::string& parent, const std::string& child,
     double x, double y, double z, double r, double p, double Y) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = parent;
        t.child_frame_id = child;

        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = z;

        tf2::Quaternion q;
        q.setRPY(r, p, Y);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
        // RCLCPP_INFO(this->get_logger(), "rotated %d deg", cnt);
    }

    void StaticTransform(const std::string& parent, const std::string& child,
     double x, double y, double z, double r, double p, double Y) 
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.frame_id = parent;
        t.child_frame_id = child;

        t.transform.translation.x = x;
        t.transform.translation.y = y;
        t.transform.translation.z = z;

        tf2::Quaternion q;
        q.setRPY(r, p, Y);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        static_tf_broadcaster_->sendTransform(t);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FramePub>();
    try {
        RCLCPP_INFO(node->get_logger(), "spin start");
        rclcpp::spin(node);
    } catch (...) {
        RCLCPP_WARN(node->get_logger(), "stoppp");
    }
    rclcpp::shutdown();
    return 0;
}
