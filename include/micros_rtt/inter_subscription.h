#ifndef MICROSRTT_INTER_SUBSCRIPTION_H
#define MICROSRTT_INTER_SUBSCRIPTION_H

#include "ros/ros.h"
#include "micros_rtt/connection_base.h"

namespace micros_rtt
{
template <class M>
class InterSubscription : public ConnectionBase
{
public:
  InterSubscription(const std::string& topic, boost::function<void(M)> fp) : ConnectionBase(topic)
  {
    callback = fp;
  }
  ~InterSubscription() {}

  void setCallback(boost::function<void(M)> fp)
  {
    callback = fp;
  }
  
  bool channelReady( ChannelElementBase::shared_ptr end_port) 
  {
    if (end_port->inputReady ()) 
    {
      return true;
    }
    return false;
  }
  
  bool call()
  {
  }
};

private:
  boost::function<void(M)> callback;

}

#endif