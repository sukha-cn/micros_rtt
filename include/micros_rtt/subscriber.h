/* 
 * File:   subscriber.h
 * Author: sukha-cn
 *
 * Created on May 28, 2015, 4:15 PM
 */

#ifndef MICROSRTT_SUBSCRIBER_HANDLE_H
#define MICROSRTT_SUBSCRIBER_HANDLE_H

#include "ros/subscriber.h"
#include "topic_manager.h"

namespace micros_rtt
{

class Subscriber 
{
public:
  Subscriber(){}
  Subscriber(ros::Subscriber ros_subscriber, ConnectionBasePtr sub_connection);
  ~Subscriber(){}
  
  template<class M> void call() 
  { 
    if(subscription)
    {
	    ROS_DEBUG("micros subscriber have subscription %s", subscription->getTopic().c_str());
  	  Subscription<M>* sub = static_cast< Subscription<M>*>(subscription.get());
  	  sub->call();
    }
  }
  
  ros::Subscriber getRosSubscriber() { return ros_sub; }
  
private:
  ConnectionBasePtr subscription;
  ros::Subscriber ros_sub;
};

}


#endif /* SUBSCRIBER_H */

