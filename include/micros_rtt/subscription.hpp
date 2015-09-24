/* 
 *  subscription.hpp - micros sub connection
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
#ifndef MICROSRTT_SUBSCRIPTION_HPP
#define MICROSRTT_SUBSCRIPTION_HPP

#include "ros/ros.h"
#include "boost/function.hpp"
#include "micros_rtt/connection_base.h"
#include "micros_rtt/oro/channel_data_element.hpp"

namespace micros_rtt
{  

template<class M>
class Subscription : public ConnectionBase
{
public:
  //typedef boost::intrusive_ptr< Subscription<M> > shared_ptr;
  Subscription(const std::string& topic) : ConnectionBase(topic) {};
  Subscription(const std::string& topic, boost::function<void(M)> fp) : ConnectionBase(topic)
  {
    callback = fp;
  }
  ~Subscription() {};
  
  void setCallback(boost::function<void(M)> fp)
  {
    callback = fp;
  }
  
  bool channelReady( ChannelElementBase::shared_ptr channel) 
  {
    if (channel && channel->inputReady())
    {
      addConnection(channel);
      return true;
    }
    return false;
  }
  
  bool mqChannelReady( ChannelElementBase::shared_ptr channel) 
  {
    if (channel && channel->inputReady())
    {
      addMQConnection(channel);
      return true;
    }
    return false;
  }
  
  bool call()
  {
    FlowStatus result;
    M sample;
    M mq_sample;
    typename ChannelElement<M>::shared_ptr input = static_cast< ChannelElement<M>* >( this->getChannelElement().get() );
    typename ChannelElement<M>::shared_ptr mq_input = static_cast< ChannelElement<M>* >( this->getMQChannelElement().get() );
    
    if (!(mq_input || input))
    {
      return false;
    }
    //if ( input ) 
    //{
    //  FlowStatus tresult = input->read(sample, false);
    //  // the result trickery is for not overwriting OldData with NoData.
    //  if (tresult == NewData) 
    //  {
    //    result = tresult;
    //    callback(sample);
    //  }
    //  // stores OldData result
    //  if (tresult > result)
    //    result = tresult;
    //}
    
    if ( mq_input ) 
    {
      FlowStatus tresult = mq_input->read(mq_sample, false);
      // the result trickery is for not overwriting OldData with NoData.
      if (tresult == NewData) 
      {
        result = tresult;
        callback(mq_sample);
      }
      // stores OldData result
      if (tresult > result)
        result = tresult;
    }
    return result;

  }
private:
  boost::function<void(M)> callback;
};
  
  
}

#endif
