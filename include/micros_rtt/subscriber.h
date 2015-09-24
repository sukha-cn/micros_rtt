/* 
 *  subscriber.h - micros topic subscriber
 *  Copyright (C) 2015 Zaile Jiang
 *  
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/

#ifndef MICROSRTT_SUBSCRIBER_HANDLE_H
#define MICROSRTT_SUBSCRIBER_HANDLE_H

#include "ros/ros.h"
#include "micros_rtt/topic_manager.h"
#include "micros_rtt/subscription.h"

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

