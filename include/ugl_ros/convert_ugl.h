#ifndef UGL_ROS_CONVERT_UGL_H
#define UGL_ROS_CONVERT_UGL_H

#include <algorithm>
#include <iterator>

#include <ros/duration.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>

#include <ugl/math/vector.h>
#include <ugl/trajectory/bezier.h>
#include <ugl/trajectory/bezier_sequence.h>
#include <ugl/trajectory/slerp_segment.h>
#include <ugl/trajectory/slerp_sequence.h>

#include <ugl_msgs/Bezier.h>
#include <ugl_msgs/BezierSequence.h>
#include <ugl_msgs/SlerpSegment.h>
#include <ugl_msgs/SlerpSequence.h>

#include "ugl_ros/convert_tf2.h"

namespace ugl_ros
{

inline
ugl::Vector<6> fromMsg(const geometry_msgs::Twist& msg)
{
    Eigen::Matrix<double,6,1> twist{};
    const Eigen::Vector3d angular{msg.angular.x , msg.angular.y , msg.angular.z};
    const Eigen::Vector3d linear{msg.linear.x , msg.linear.y, msg.linear.z};
    twist << angular, linear;
    return twist;
}

inline
geometry_msgs::Twist& toMsg(const ugl::Vector<6>& in, geometry_msgs::Twist& out)
{
    tf2::toMsg(in.segment<3>(0), out.angular);
    tf2::toMsg(in.segment<3>(3), out.linear);
    return out;
}

inline
geometry_msgs::Twist toMsg(const ugl::Vector<6>& twist)
{
    geometry_msgs::Twist msg{};
    toMsg(twist, msg);
    return msg;
}

template<int degree>
ugl_msgs::Bezier toMsg(const ugl::trajectory::Bezier<degree>& bezier)
{
    ugl_msgs::Bezier msg{};
    msg.duration = ros::Duration{bezier.duration()};
    std::transform(std::begin(bezier.points()),
                   std::end(bezier.points()),
                   std::back_inserter(msg.points),
                   [](const auto& vec){ return tf2::toMsg<geometry_msgs::Point>(vec); });
    return msg;
}

template<int degree>
ugl_msgs::BezierSequence toMsg(const ugl::trajectory::BezierSequence<degree>& bezier_seq)
{
    ugl_msgs::BezierSequence msg{};
    std::transform(std::begin(bezier_seq.segments()),
                   std::end(bezier_seq.segments()),
                   std::back_inserter(msg.beziers),
                   [](const auto& bezier){ return ugl_ros::toMsg(bezier); });
    return msg;
}

ugl_msgs::SlerpSegment toMsg(const ugl::trajectory::SlerpSegment& segment)
{
    ugl_msgs::SlerpSegment msg{};
    msg.duration = ros::Duration{segment.duration()};
    msg.start = tf2::toMsg(segment.start());
    msg.end = tf2::toMsg(segment.end());
    return msg;
}

ugl_msgs::SlerpSequence toMsg(const ugl::trajectory::SlerpSequence& sequence)
{
    ugl_msgs::SlerpSequence msg{};
    std::transform(std::begin(sequence.segments()),
                   std::end(sequence.segments()),
                   std::back_inserter(msg.segments),
                   [](const auto& segment){ return ugl_ros::toMsg(segment); });
    return msg;
}

} // namespace ugl_ros

#endif // UGL_ROS_CONVERT_UGL_H
