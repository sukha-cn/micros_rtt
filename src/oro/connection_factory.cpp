/* 
 *  connection_factory.cpp - micros connection factory
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
#include "micros_rtt/oro/connection_factory.hpp"

namespace micros_rtt
{
  
bool ConnFactory::createAndCheckConnection(ConnectionBasePtr publication, 
          ConnectionBasePtr subscription, ChannelElementBase::shared_ptr channel_input)
{
  // Register the subscription to the publication.
  if ( publication->addConnection(channel_input) ) 
  {
    ROS_DEBUG("micros connection factory adds input_end of channel to publication:%s.", publication->getTopic().c_str());
    // notify input that the connection is now complete.
    if ( subscription->channelReady( channel_input->getOutputEndPoint() ) == false ) 
    {
      ROS_WARN("micros connection factory checks channel ready failed, subscription:%s", subscription->getTopic().c_str());
      return false;
    }
    return true;
  }
  // setup failed.
  return false;
}

bool ConnFactory::createAndCheckStream(ConnectionBasePtr connection, 
        ChannelElementBase::shared_ptr chan, bool is_sender)
{
  if (is_sender)
  {
    if ( connection->addMQConnection(chan) ) 
    {
      ROS_DEBUG("micros connection factory adds input_end of stream to publication:%s.", connection->getTopic().c_str());
      return true;
    }
    // setup failed.
    ROS_WARN("micros connection factory fail to add input_end of stream to publication:%s.", connection->getTopic().c_str());
    return false;
  }
  else
  {
    if ( connection->mqChannelReady( chan->getOutputEndPoint() ) == true ) 
    {
      ROS_DEBUG("micros connection factory adds output_end of stream to subscription:%s.", connection->getTopic().c_str());
      return true;
    }
    // setup failed: manual cleanup.
    chan = 0; 
    ROS_WARN("micros connection factory fail to add output_end of stream to subscription:%s.", connection->getTopic().c_str());
    return false;
  }
}

}
