#include "robot_frame.h"

using namespace std::chrono_literals;


RobotFrame::RobotFrame(): Node("robot_frame"), cnt(0),
    base_("base", "world"),
    link0_("link0", base_),
    link1_("link1", link0_),
    link2_("link2", link1_)
{
    timer_ = this->create_wall_timer(100ms, std::bind(&RobotFrame::TimerCallback, this));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);

    base_.SetOffset_XYZ(0, 0, 1);
    PublishStaticTransform(base_);
    
    link0_.SetOffset_XYZ(0, 1, 0);
    link0_.SetOffset_RPY(0, 0, 0);
    PublishStaticTransform(link0_);

    link1_.SetOffset_XYZ(0, 1, 0);
    PublishStaticTransform(link1_);
}

void RobotFrame::PublishTransform(FrameUnit& frame)
{
    frame.GetTF().header.stamp = this->get_clock()->now();
    tf_broadcaster_->sendTransform(frame.GetTF());
}

void RobotFrame::PublishStaticTransform(FrameUnit& frame)
{
    frame.GetTF().header.stamp = this->get_clock()->now();
    static_tf_broadcaster_->sendTransform(frame.GetTF());
}

void RobotFrame::TimerCallback()
    {
        cnt += 1;
        link0_.MakeTF_RPYdeg(10*sin(tf2Radians(cnt)), 0, 0);
        PublishTransform(link0_);
    }


int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotFrame>();
    try {
        RCLCPP_INFO(node->get_logger(), "spin start");
        rclcpp::spin(node);
    } catch (...) {
        RCLCPP_WARN(node->get_logger(), "stoppp");
    }
    rclcpp::shutdown();
    return 0;
}
