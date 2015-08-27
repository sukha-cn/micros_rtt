/* 
 * File:   subscriber.cpp
 * Author: sukha-cn
 * 
 * Created on June 3, 2015, 9:59 AM
 */

#include "micros_rtt/subscriber.h"

namespace micros_rtt
{
 
Subscriber::Subscriber() 
{ 
}
 
Subscriber::Subscriber(ConnectionBasePtr sub_connection, ros::Subscriber ros_subscriber) 
{
  subscription = sub_connection;
  ros_sub = ros_subscriber;
}

Subscriber::~Subscriber() 
{ 
}

}
