#ifndef ROSRTT_NODE_HANDLE_H
#define ROSRTT_NODE_HANDLE_H

#include "ros/node_handle.h"
#include "publisher.h"
#include "subscriber.h"
#include "topic_manager.h"

namespace rosrtt {

class NodeHandle
{
public:
  NodeHandle(const std::string& ns = std::string(), const ros::M_string& remappings = ros::M_string());
//  NodeHandle(const NodeHandle& rhs);
//  NodeHandle(const NodeHandle& parent, const std::string& ns);
//  NodeHandle(const NodeHandle& parent, const std::string& ns, const ros::M_string& remappings);
  ~NodeHandle();

  //NodeHandle& operator=(const NodeHandle& rhs);

  void setCallbackQueue(ros::CallbackQueueInterface* queue)
  {
    ros_nh.setCallbackQueue(queue);
  }

  ros::CallbackQueueInterface* getCallbackQueue() const
  {
    return ros_nh.getCallbackQueue();
  }

  const std::string& getNamespace() const 
  {
    return ros_nh.getNamespace();
  }

  const std::string& getUnresolvedNamespace()
  {
    return ros_nh.getUnresolvedNamespace();
  }

  std::string resolveName(const std::string& name, bool remap = true) const
  {
    return ros_nh.getUnresolvedNamespace();
  }


  template <class M>
  Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch = false)
  {
    ros::Publisher ros_pub = ros_nh.advertise<M>(topic, queue_size, latch);
    ROS_INFO("advertise.");
    
    if (ros_pub) 
    {
      ConnectionBasePtr pub_connection = TopicManager::instance()->advertise<M>(topic);
      if (pub_connection)
      {
        ROS_INFO("publish successes.");
        Publisher pub(pub_connection, ros_pub);
        return pub;
      }
    }

    return Publisher();
  }

  template <class M>
  Subscriber subscribe(const std::string &topic, uint32_t queue_size, void(*fp)(M),
             const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    // this will lead local messages to be double received, not resolved.
    ros::Subscriber ros_sub = ros_nh.subscribe(topic, queue_size, fp, transport_hints);
    ROS_INFO("subscribe.");
    if (ros_sub) 
    {
      ROS_INFO("ros subscribe successful.");
      ConnectionBasePtr sub_connection = TopicManager::instance()->subscribe<M>(topic, fp);
      if (sub_connection)
      {
        ROS_INFO("subscribe successes.");
        Subscriber sub(sub_connection, ros_sub);
        return sub;
      }
    }

    return Subscriber();
  }

private:
  ros::NodeHandle ros_nh;

};

}

#endif
