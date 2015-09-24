/* 
 *  publication.hpp - micros pub connection
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
#ifndef MICROSRTT_PUBLICATION_HPP
#define MICROSRTT_PUBLICATION_HPP

#include "ros/ros.h"
#include "micros_rtt/connection_base.h"
#include "micros_rtt/oro/channel_data_element.hpp"

namespace micros_rtt
{

template <class M>
class Publication : public ConnectionBase
{
public:
  typedef boost::shared_ptr< Publication<M> > shared_ptr;
  
  Publication(const std::string& topic) : ConnectionBase(topic) {}
  ~Publication() {}

  bool publish(M message)
  {
    typename ChannelElement<M>::shared_ptr output
                = boost::static_pointer_cast< ChannelElement<M> >(this->getChannelElement());
    typename ChannelElement<M>::shared_ptr mq_output
                = boost::static_pointer_cast< ChannelElement<M> >(this->getMQChannelElement());
    
    if (!(mq_output || output))
    {
      return false;
    }
    if (mq_output)
    {
      mq_output->write(message);
    }
    if (output)
    {
      output->write(message);
    }
    return true;
  }

  virtual bool channelReady(ChannelElementBase::shared_ptr channel) {return false;}

  virtual bool mqChannelReady(ChannelElementBase::shared_ptr channel) {return false;}

};

}

#endif
