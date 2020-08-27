#ifndef UGL_ROS_CONVERT_TF2_H
#define UGL_ROS_CONVERT_TF2_H

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>
#include <ugl/lie_group/rotation.h>

namespace tf2
{

inline
ugl::UnitQuaternion fromMsg(const geometry_msgs::Quaternion& in)
{
    return ugl::UnitQuaternion{in.w, in.x, in.y, in.z};
}

inline
ugl::Vector3 fromMsg(const geometry_msgs::Point& in)
{
    return ugl::Vector3{in.x, in.y, in.z};
}

inline
ugl::Vector3 fromMsg(const geometry_msgs::Vector3& in)
{
    return ugl::Vector3{in.x, in.y, in.z};
}

template<typename MsgType>
inline
MsgType toMsg(const ugl::Vector3& in)
{
    MsgType msg;
    msg.x = in.x();
    msg.y = in.y();
    msg.z = in.z();
    return msg;
}

inline
geometry_msgs::Quaternion toMsg(const ugl::UnitQuaternion& in)
{
    geometry_msgs::Quaternion msg;
    msg.x = in.x();
    msg.y = in.y();
    msg.z = in.z();
    msg.w = in.w();
    return msg;
}

inline
geometry_msgs::Quaternion toMsg(const ugl::lie::Rotation& in)
{
    return tf2::toMsg(in.to_quaternion());
}

inline
geometry_msgs::Vector3& toMsg(const ugl::Vector3& in, geometry_msgs::Vector3& out)
{
    out.x = in.x();
    out.y = in.y();
    out.z = in.z();
    return out;
}

inline
geometry_msgs::Point& toMsg(const ugl::Vector3& in, geometry_msgs::Point& out)
{
    out.x = in.x();
    out.y = in.y();
    out.z = in.z();
    return out;
}

inline
geometry_msgs::Quaternion& toMsg(const ugl::UnitQuaternion& in, geometry_msgs::Quaternion& out)
{
    out = tf2::toMsg(in);
    return out;
}

inline
geometry_msgs::Quaternion& toMsg(const ugl::lie::Rotation& in, geometry_msgs::Quaternion& out)
{
    out = tf2::toMsg(in);
    return out;
}

} // namespace tf2

#endif // UGL_ROS_CONVERT_TF2_H
