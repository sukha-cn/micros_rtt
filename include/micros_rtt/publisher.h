#ifndef MICROSRTT_PUBLISHER_HANDLE_H
#define MICROSRTT_PUBLISHER_HANDLE_H

#include "ros/publisher.h"
#include "topic_manager.h"

namespace micros_rtt
{

class Publisher 
{
public:
  Publisher(){}
  Publisher(ros::Publisher ros_publisher, ConnectionBasePtr pub_connection);
  ~Publisher(){}
  
  template <class M>
  void publish(M message)
  {
    if (publication)
    {
      typename Publication<M>::shared_ptr output = boost::static_pointer_cast< Publication<M> >(publication.get());
      if(!output->publish(message))
        ROS_WARNING("micros publish failed");
    }
    else 
    {
      ROS_WARNING("micros publisher can't publish message with no connection.");
    }

  }
  
  ros::Publisher getRosPublisher() {return ros_pub;}
  
private:
  ros::Publisher ros_pub;
  ConnectionBasePtr publication;
};

}

#endif
