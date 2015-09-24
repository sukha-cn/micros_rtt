/* 
 *  connection_base.h - micros connection base
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
#ifndef MICROSRTT_CONNECTION_BASE_H
#define MICROSRTT_CONNECTION_BASE_H

#include "micros_rtt/oro/channel_element_base.hpp"

namespace micros_rtt
{
class ConnectionBase;
typedef boost::shared_ptr<ConnectionBase> ConnectionBasePtr;
typedef std::vector<ConnectionBasePtr> V_ConnectionBase;

class ConnectionBase
{
public:
  typedef boost::intrusive_ptr<ConnectionBase> shared_ptr;
  
private:
  std::string topic_;
  ChannelElementBase::shared_ptr channel_element;
  ChannelElementBase::shared_ptr mq_channel_element;

public:
  ConnectionBase() {}
  ConnectionBase(std::string topic) : topic_(topic) {}
  ~ConnectionBase() {}
  
  std::string getTopic() {return topic_;}
  
  ChannelElementBase::shared_ptr getChannelElement() {return channel_element;}
  ChannelElementBase::shared_ptr getMQChannelElement() {return mq_channel_element;}
  
  bool addConnection(ChannelElementBase::shared_ptr channel) 
  {
    channel_element = channel;
    return true;
  }
  bool addMQConnection(ChannelElementBase::shared_ptr channel) 
  {
    mq_channel_element = channel;
    return true;
  }

  virtual bool channelReady(ChannelElementBase::shared_ptr channel) = 0;
  virtual bool mqChannelReady(ChannelElementBase::shared_ptr channel) = 0;
};

}
#endif
