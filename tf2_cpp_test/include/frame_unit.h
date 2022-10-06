#ifndef TF2_CPP_TEST_FRAME_UNIT_H
#define TF2_CPP_TEST_FRAME_UNIT_H

#include <memory>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "tf2/LinearMath/Quaternion.h"

class FrameUnit
{
public:
    FrameUnit() = delete;
    FrameUnit(const std::string& name, const std::string& parent);
    FrameUnit(const std::string& name, const FrameUnit& parent);

    void SetOffset_XYZ(double x=0.0, double y=0.0, double z=0.0);
    void SetOffset_RPY(double r=0.0, double p=0.0, double y=0.0);
    void SetOffset_RPYdeg(double r=0.0, double p=0.0, double y=0.0);

    const std::string& GetName() const { return t_.child_frame_id; }
    std::string& GetParentName() { return t_.header.frame_id; }
    geometry_msgs::msg::TransformStamped& GetTF() { return t_; }

    void MakeTF_XYZ(double x=0.0, double y=0.0, double z=0.0);
    void MakeTF_RPY(double roll=0.0, double pitch=0.0, double yaw=0.0);
    void MakeTF_RPYdeg(double roll=0.0, double pitch=0.0, double yaw=0.0);
    void MakeTF(double x=0.0, double y=0.0, double z=0.0,
        double roll=0.0, double pitch=0.0, double yaw=0.0);    


private:
    double offset_x_ = 0;
    double offset_y_ = 0;
    double offset_z_ = 0;
    double offset_roll_ = 0;
    double offset_pitch_ = 0;
    double offset_yaw_ = 0;
    geometry_msgs::msg::TransformStamped t_;
};

#endif // TF2_CPP_TEST_FRAME_UNIT_H 