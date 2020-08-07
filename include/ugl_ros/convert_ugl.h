#pragma once

#include <algorithm>
#include <iterator>

#include <ros/duration.h>

#include <geometry_msgs/Point.h>

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


}