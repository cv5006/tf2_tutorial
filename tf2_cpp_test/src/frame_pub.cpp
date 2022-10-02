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

using namespace std::chrono_literals;

class FramePub : public rclcpp::Node
{
public:
    FramePub(): Node("frame_pub"), cnt(0)
    {
        timer_ = this->create_wall_timer(10ms, std::bind(&FramePub::TimerCallback, this));
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
        tf_broadcaster2_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

private:
    int cnt;
    rclcpp::TimerBase::SharedPtr timer_;
    void TimerCallback()
    {
        cnt += 1;
        Transform();
        if (cnt % 3 == 0) {
            Transform2();
        }
        try {
        } catch (...) {
          RCLCPP_ERROR(this->get_logger(), "wtf");
          return;
        }
    }

    void Transform() {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "world";
        t.child_frame_id = "base";

        tf2::Quaternion q;
        q.setRPY(0, 0, tf2Radians(cnt));
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(t);
        RCLCPP_INFO(this->get_logger(), "rotated %d deg", cnt);
    }

    void Transform2()
    {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = this->get_clock()->now();
        t.header.frame_id = "base";
        t.child_frame_id = "arm";

        t.transform.translation.x = 1.0 * cos(tf2Radians(cnt)*2);
        t.transform.translation.z = 1.0 * sin(tf2Radians(cnt)*2);

        tf2::Quaternion q;
        q.setRPY(tf2Radians(cnt)*2, 0, 0);
        t.transform.rotation.x = q.x();
        t.transform.rotation.y = q.y();
        t.transform.rotation.z = q.z();
        t.transform.rotation.w = q.w();
        tf_broadcaster2_->sendTransform(t);
    }

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster2_;
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
