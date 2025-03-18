#ifndef ROS_NODE_BASE_HPP
#define ROS_NODE_BASE_HPP

#include <rclcpp/rclcpp.hpp>

#ifdef USE_LIFECYCLE_NODES
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#endif

// 🚀 Define RosNodeBase Alias
#ifdef USE_LIFECYCLE_NODES
using RosNodeBase = rclcpp_lifecycle::LifecycleNode;
#else
using RosNodeBase = rclcpp::Node;
#endif

#endif // ROS_NODE_BASE_HPP