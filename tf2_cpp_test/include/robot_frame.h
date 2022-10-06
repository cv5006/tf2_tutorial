#include "math.h"

#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/static_transform_broadcaster.h"

#include "frame_unit.h"


class RobotFrame : public rclcpp::Node
{
public:
    RobotFrame();

private:
    int cnt;
    rclcpp::TimerBase::SharedPtr timer_;
    void TimerCallback();

    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    void PublishTransform(FrameUnit& frame);

    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    void PublishStaticTransform(FrameUnit& frame);

private:
    FrameUnit base_;
    FrameUnit link0_;
    FrameUnit link1_;
    FrameUnit link2_;
    FrameUnit link3_;
    FrameUnit link4_;
    FrameUnit link5_;
    FrameUnit link6_;
};
