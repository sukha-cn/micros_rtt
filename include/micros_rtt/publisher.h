/* 
 *  publisher.h - micros topic publisher
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
#ifndef MICROSRTT_PUBLISHER_HANDLE_H
#define MICROSRTT_PUBLISHER_HANDLE_H

#include "ros/ros.h"
#include "micros_rtt/topic_manager.h"
#include "micros_rtt/publication.h"

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
      typename Publication<M>::shared_ptr output = boost::static_pointer_cast< Publication<M> >(publication);
      if(!output->publish(message))
        ROS_WARN("micros publish failed");
    }
    else 
    {
      ROS_WARN("micros publisher can't publish message with no connection.");
    }

  }
  
  ros::Publisher getRosPublisher() {return ros_pub;}
  
private:
  ros::Publisher ros_pub;
  ConnectionBasePtr publication;
};

}

#endif
