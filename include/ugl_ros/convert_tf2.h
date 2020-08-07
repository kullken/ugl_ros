#pragma once

#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ugl/math/vector.h>
#include <ugl/math/matrix.h>
#include <ugl/math/quaternion.h>

namespace tf2
{

inline
ugl::Rotation fromMsg(const geometry_msgs::Quaternion& in)
{
    ugl::UnitQuaternion q;
    q.x() = in.x;
    q.y() = in.y;
    q.z() = in.z;
    q.w() = in.w;
    return q.toRotationMatrix();
}

inline
ugl::Vector3 fromMsg(const geometry_msgs::Point& in)
{
    ugl::Vector3 out;
    out.x() = in.x;
    out.y() = in.y;
    out.z() = in.z;
    return out;
}

inline
ugl::Vector3 fromMsg(const geometry_msgs::Vector3& in)
{
    ugl::Vector3 out;
    out.x() = in.x;
    out.y() = in.y;
    out.z() = in.z;
    return out;
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
geometry_msgs::Quaternion toMsg(const ugl::Rotation& in)
{
    return tf2::toMsg(ugl::UnitQuaternion(in));
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
geometry_msgs::Quaternion& toMsg(const ugl::Rotation& in, geometry_msgs::Quaternion& out)
{
    out = tf2::toMsg(in);
    return out;
}

}
