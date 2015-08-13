#ifndef HPCLRTT_CONNECTION_BASE_H
#define	HPCLRTT_CONNECTION_BASE_H

#include "hpcl_rtt/oro/channel_element_base.hpp"

namespace hpcl_rtt
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
  bool is_intraprocess_;
  ChannelElementBase::shared_ptr channel_element;

public:
  ConnectionBase(std::string topic, bool is_intraprocess) : topic_(topic), is_intraprocess_(is_intraprocess) {}
  ~ConnectionBase() {};
  
  std::string getTopic() {return topic_;}
  
  ChannelElementBase::shared_ptr getChannelElement() {return channel_element;}
  
  bool addConnection(ChannelElementBase::shared_ptr channel) {channel_element = channel;};
  
  bool isIntraprocess() {return is_intraprocess_;}
};

}
#endif
