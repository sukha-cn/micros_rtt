#ifndef MICROSRTT_PUBLISHER_HANDLE_H
#define MICROSRTT_PUBLISHER_HANDLE_H

#include "ros/publisher.h"
#include "topic_manager.h"

namespace micros_rtt
{

class Publisher 
{
public:
  Publisher();
  Publisher(ros::Publisher ros_publisher);
  Publisher(ConnectionBasePtr pub_connection, ros::Publisher ros_publisher);
  ~Publisher();
  
  template <class M>
  void publish(M message)
  {
    if (publication)
    {
      //typename Publication<M>::shared_ptr output = static_cast< Publication<M>* >( pub.get() );
      typename Publication<M>::shared_ptr output = boost::static_pointer_cast< Publication<M> >(publication);
      if(!output->publish(message))
        ROS_WARNING("publish failed");
    }
    else 
    {
      //ROS_INFO("ros publishing.");
      ros_pub.publish<M>(message);
    }
    
    return;
  }
  
private:
  ros::Publisher ros_pub;
  ConnectionBasePtr publication;
};

}

#endif
