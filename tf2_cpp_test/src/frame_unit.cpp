#include "frame_unit.h"

FrameUnit::FrameUnit(const std::string& name, const std::string& parent)
{
    t_.child_frame_id = name;
    t_.header.frame_id = parent;
}

FrameUnit::FrameUnit(const std::string& name, const FrameUnit& parent)
{
    t_.child_frame_id = name;
    t_.header.frame_id = parent.GetName();
}

void FrameUnit::SetOffset_XYZ(double x, double y, double z)
{
    offset_x_ = x;
    offset_y_ = y;
    offset_z_ = z;
    MakeTF();
}

void FrameUnit::SetOffset_RPY(double r, double p, double y)
{
    offset_roll_  = r;
    offset_pitch_ = p;
    offset_yaw_   = y;
    MakeTF();
}

void FrameUnit::SetOffset_RPYdeg(double r, double p, double y)
{
    offset_roll_  = tf2Radians(r);
    offset_pitch_ = tf2Radians(p);
    offset_yaw_   = tf2Radians(y);
    MakeTF();
}

void FrameUnit::MakeTF_XYZ(double x, double y, double z)
{
    MakeTF(x, y, z, 0.0, 0.0, 0.0);
}

void FrameUnit::MakeTF_RPY(double roll, double pitch, double yaw)
{
    MakeTF(0.0, 0.0, 0.0, roll, pitch, yaw);
}

void FrameUnit::MakeTF_RPYdeg(double roll, double pitch, double yaw)
{
    MakeTF(0.0, 0.0, 0.0, tf2Radians(roll), tf2Radians(pitch), tf2Radians(yaw));
}

void FrameUnit::MakeTF(double x, double y, double z,
        double roll, double pitch, double yaw)
{
    t_.transform.translation.x = offset_x_ + x;
    t_.transform.translation.y = offset_y_ + y;
    t_.transform.translation.z = offset_z_ + z;

    tf2::Quaternion q;
    q.setRPY(offset_roll_+roll, offset_pitch_+pitch, offset_yaw_+yaw);
    t_.transform.rotation.x = q.x();
    t_.transform.rotation.y = q.y();
    t_.transform.rotation.z = q.z();
    t_.transform.rotation.w = q.w();
}