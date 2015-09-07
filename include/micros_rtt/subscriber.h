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
  Subscriber();
  Subscriber(ConnectionBasePtr sub_connection, ros::Subscriber ros_subscriber);
  ~Subscriber();
  
  template<class M> void call() 
  { 
    if(subscription)
    {
	  ROS_INFO("have subscription %s", subscription->getTopic().c_str());
	  if (subscription->isInterprocess())
	  {
        InterSubscription<M>* sub = static_cast< InterSubscription<M>* >(subscription.get ());
        sub->call();
	  }
	  else
	  {
	    Subscription<M>* sub = static_cast< Subscription<M>*>(subscription.get());
	    sub->call();
	  }		  
    }
  }
  
private:
  ConnectionBasePtr subscription;
  ros::Subscriber ros_sub;
};

}


#endif /* SUBSCRIBER_H */

