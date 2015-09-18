#ifndef MICROSRTT_SUBSCRIPTION_H
#define MICROSRTT_SUBSCRIPTION_H

#include "ros/ros.h"
#include "boost/function.hpp"
#include "micros_rtt/connection_base.hpp"
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
        //callback(sample);
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
