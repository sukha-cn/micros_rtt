#ifndef ROSRTT_CONNECTION_BASE_H
#define	ROSRTT_CONNECTION_BASE_H

#include "ros_rtt/oro/channel_element_base.hpp"

namespace rosrtt
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

public:
  ConnectionBase(std::string topic) : topic_(topic) {}
  ~ConnectionBase() {};
  
  std::string getTopic() {return topic_;}
  
  ChannelElementBase::shared_ptr getChannelElement() {return channel_element;}
  
  bool addConnection(ChannelElementBase::shared_ptr channel) {channel_element = channel;};
};

}
#endif