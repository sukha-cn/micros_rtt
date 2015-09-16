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
      
      //ros has done the necessary parameter check, so we know the topic is new.
      //and we know the subscribers (if there are) have the same data type.
      ConnectionBasePtr pub_connection(new Publication<M>(topic));
      TopicManager::instance()->addPubConnection(pub_connection);
        
      //create remote message queue for inter-process transport.
      ConnFactory::createStream<M>(pub_connection, true);

      //check if local subscription exists and make the connection.
      ConnectionBasePtr local_sub = TopicManager::instance()->findSubConnection(topic);
      if (local_sub)
      {
        //make local connection
        ConnFactory::createConnection<M>(pub_connection, local_sub);
      }
      
      Publisher pub(ros_pub, pub_connection);
      return pub;
    }
    else 
    {
      ROS_WARN("micros publishes topic %s on ros failed", topic);
      return Publisher();
    }
  }

  /**
   * \brief Subscribe to a topic, version for bare function, compatible with ros
   *
   * The detail of this method will coming soon.
   *
   * This version of subscribe is a convenience function for using bare functions, and can be used like so:
\verbatim
void callback(const std_msgs::Empty::ConstPtr& message)
{
}

ros::Subscriber sub = handle.subscribe("my_topic", 1, callback);
\endverbatim
   *
   * \param M [template] M here is the callback parameter type (e.g. const boost::shared_ptr<M const>& or const M&), \b not the message type, and should almost always be deduced
   * \param topic Topic to subscribe to
   * \param queue_size Number of incoming messages to queue up for
   * processing (messages in excess of this queue capacity will be
   * discarded).
   * \param fp Function pointer to call when a message has arrived
   * \param transport_hints a TransportHints structure which defines various transport-related options
   * \return On success, a Subscriber that, when all copies of it go out of scope, will unsubscribe from this topic.
   * On failure, an empty Subscriber which can be checked with:
\verbatim
if (handle)
{
...
}
\endverbatim
   *  \throws InvalidNameException If the topic name begins with a tilde, or is an otherwise invalid graph resource name
   *  \throws ConflictingSubscriptionException If this node is already subscribed to the same topic with a different datatype
   */
  template <class M>
  Subscriber subscribe(const std::string &topic, uint32_t queue_size, void(*fp)(M),
             const ros::TransportHints& transport_hints = ros::TransportHints())
  {
    //try to subscribe the topic in ros, it will also make the necessary parameter check.
    //we don't use ros for transport, so register NULL.
    ros::Subscriber ros_sub = ros_nh.subscribe(topic, queue_size, NULL, transport_hints);
    
    if (ros_sub) 
    {
      //topic get subscribed in ros, do the subscribe work in micros
      ROS_INFO("micros has subscribed topic %s on ros with %s.", topic, ros_sub.getTopic().c_str());

      //ros has done the necessary parameter check, so we know the topic is new.
      //and we know the publisher (if there are) have the same data type.
      ConnectionBasePtr sub_connection(new subscription(topic, fp));
      TopicManager::addSubConnection(sub_connection);
      
      //create remote message queue for inter-process transport.
      ConnFactory::createStream<M>(sub_connection, false);
      
      //check if local subscription exists and make the connection.
      ConnectionBasePtr local_pub = TopicManager::instance()->findPubConnection(topic);
      if (local_pub)
      {
        //make local connection
        ConnFactory::createConnection<M>(local_pub, sub_connection);
      }
      
      Subscriber sub(sub_connection, ros_sub);
      return sub;
    }
    else 
    {
      ROS_WARN("micros subscribes topic %s on ros failed", topic);
      return Subscriber();
    }

  }

private:
  ros::NodeHandle ros_nh;

};

}

#endif
