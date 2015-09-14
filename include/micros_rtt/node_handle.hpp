#ifndef MICROSRTT_NODE_HANDLE_HPP
#define MICROSRTT_NODE_HANDLE_HPP

#include "ros/node_handle.h"
#include "publisher.h"
#include "subscriber.h"
#include "topic_manager.h"

namespace micros_rtt {

class NodeHandle
{
public:
  NodeHandle(const std::string& ns = std::string(), const ros::M_string& remappings = ros::M_string())
  {
    //get ros node handle first
    ros_nh = ros::NodeHandle(ns, remappings);
    //maybe need other initialization later
    
  }
  ~NodeHandle()
  {
    //maybe something need to release the resources
  }

  //ros node handle method, leave for compatibility
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


  //micros node handle method from here
  
  /**
  * \brief Advertise a topic, simple version, compatible with ros
  *
  * The detail of this method will coming soon.
  *
  * \param topic Topic to advertise on
  *
  * \param queue_size Maximum number of outgoing messages to be
  * queued for delivery to subscribers
  *
  * \param latch (optional) If true, the last message published on
  * this topic will be saved and sent to new subscribers when they
  * connect
  *
  * \return On success, a Publisher that, when it goes out of scope,
  * will automatically release a reference on this advertisement.  On
  * failure, an empty Publisher.
  *
  * \throws InvalidNameException If the topic name begins with a
  * tilde, or is an otherwise invalid graph resource name, or is an
  * otherwise invalid graph resource name
  */
  template <class M>
  Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch = false)
  {
    //try to publish the topic in ros first, it will also make the necessary parameter check.
    ros::Publisher ros_pub = ros_nh.advertise<M>(topic, queue_size, latch);

    if (ros_pub) 
    {
      //topic get published in ros, do the advertise work in micros
      ROS_INFO("micros has published topic %s on ros with %s.", topic, ros_pub.getTopic().c_str());
      
      ConnectionBasePtr pub_connection = TopicManager::instance()->advertise<M>(topic);
      if (pub_connection)
      {
        Publisher pub(pub_connection, ros_pub);
        return pub;
      }
    }
    else 
    {
      ROS_WARN("micros publishes topic %s on ros failed", topic);
      return Publisher();
    }
  }

  template <class M>
  Subscriber subscribe(const std::string &topic, uint32_t queue_size, void(*fp)(M),
             const ros::TransportHints& transport_hints = ros::TransportHints(), bool is_interprocess = false)
  {
    // this will lead local messages to be double received, not resolved.
    ros::Subscriber ros_sub = ros_nh.subscribe(topic, queue_size, fp, transport_hints);
    if (ros_sub) 
    {
      ROS_INFO("ros subscribe successful.");
      ConnectionBasePtr sub_connection = TopicManager::instance()->subscribe<M>(topic, fp, is_interprocess);
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
