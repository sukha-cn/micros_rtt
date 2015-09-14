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
  bool is_interprocess_;
  ChannelElementBase::shared_ptr channel_element;

public:
  ConnectionBase() {}
  ConnectionBase(std::string topic, bool is_interprocess = false) : topic_(topic), is_interprocess_(is_interprocess) {}
  ~ConnectionBase() {}
  
  std::string getTopic() {return topic_;}
  
  ChannelElementBase::shared_ptr getChannelElement() {return channel_element;}
  
  bool addConnection(ChannelElementBase::shared_ptr channel) {channel_element = channel;}
  
  bool isInterprocess() {return is_interprocess_;}

  virtual bool channelReady(ChannelElementBase::shared_ptr channel){}
};

}
#endif
