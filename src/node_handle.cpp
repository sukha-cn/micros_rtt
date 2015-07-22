#include "hpcl_rtt/node_handle.h"

namespace hpcl_rtt
{

NodeHandle::NodeHandle(const std::string& ns, const ros::M_string& remappings)
{
  ros_nh = ros::NodeHandle(ns, remappings);
}

//NodeHandle::NodeHandle(const NodeHandle& parent, const std::string& ns)
//{
//  ros_nh = ros::NodeHandle(parent, ns);
//}
//
//NodeHandle::NodeHandle(const NodeHandle& parent, const std::string& ns, const ros::M_string& remappings)
//{
//  ros_nh = ros::NodeHandle(parent, ns, remappings);
//}
//
//NodeHandle::NodeHandle(const NodeHandle& rhs)
//{
//  ros_nh = ros::NodeHandle(rhs);
//}

NodeHandle::~NodeHandle()
{
}


}
