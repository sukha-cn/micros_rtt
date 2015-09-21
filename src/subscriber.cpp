/* 
 * File:   subscriber.cpp
 * Author: sukha-cn
 * 
 * Created on June 3, 2015, 9:59 AM
 */

#include "micros_rtt/subscriber.h"

namespace micros_rtt
{
 
Subscriber::Subscriber(ros::Subscriber ros_subscriber, ConnectionBasePtr sub_connection) 
{
  subscription = sub_connection;
  ros_sub = ros_subscriber;
}

}
