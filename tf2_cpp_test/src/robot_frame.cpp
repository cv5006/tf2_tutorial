#include "robot_frame.h"

using namespace std::chrono_literals;


RobotFrame::RobotFrame(): Node("robot_frame"), cnt(0),
    base_("base", "world"),
    link0_("link0", base_),
    link1_("link1", link0_),
    link2_("link2", link1_),
    link3_("link3", link2_),
    link4_("link4", link3_),
    link5_("link5", link4_),
    link6_("link6", link5_)
{
    timer_ = this->create_wall_timer(10ms, std::bind(&RobotFrame::TimerCallback, this));
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    static_tf_broadcaster_ = std::make_unique<tf2_ros::StaticTransformBroadcaster>(*this);


    base_.SetOffset_XYZ(0, 0, 1);
    PublishStaticTransform(base_);
    
    link0_.SetOffset_XYZ(0, 1, 0);
    link0_.SetOffset_RPY(0, 0, 0);
    PublishStaticTransform(link0_);

    link1_.SetOffset_XYZ(0, 1, 0);
    PublishStaticTransform(link1_);

    link2_.SetOffset_XYZ(1, 0, 0);
    link2_.SetOffset_RPY(M_PI/2, M_PI/2, 0);
    PublishStaticTransform(link2_);

    link3_.SetOffset_XYZ(0.5, 0, 0);
    PublishStaticTransform(link3_);

    link4_.SetOffset_XYZ(1, 0, 0);
    PublishStaticTransform(link4_);

    link5_.SetOffset_XYZ(1, 0, 0);
    link2_.SetOffset_RPY(0, 0, M_PI/2);
    PublishStaticTransform(link5_);

    link6_.SetOffset_XYZ(0.1, 0, 0);
    PublishStaticTransform(link6_);

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
    link3_.MakeTF_RPYdeg(0, 0, 20*sin(tf2Radians(cnt)));
    PublishTransform(link3_);
    link4_.MakeTF_RPYdeg(0, 0, 15*(sin(tf2Radians(cnt))-1));
    PublishTransform(link4_);
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
