/* 
 * File:   subscriber.h
 * Author: hpcl
 *
 * Created on May 28, 2015, 4:15 PM
 */

#ifndef ROSRTT_SUBSCRIBER_HANDLE_H
#define	ROSRTT_SUBSCRIBER_HANDLE_H

#include "ros/subscriber.h"
#include "topic_manager.h"

namespace rosrtt
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
      Subscription<M>* sub = static_cast< Subscription<M>* >(subscription.get ());
      sub->call();
    }
  }
  
private:
  ConnectionBasePtr subscription;
  ros::Subscriber ros_sub;
};

}


#endif	/* SUBSCRIBER_H */

