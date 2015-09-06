#ifndef MICROSRTT_INTER_PUBLICATION_H
#define MICROSRTT_INTER_PUBLICATION_H

#include "oro/channel_data_element.hpp"
#include "ros/ros.h"
#include "micros_rtt/connection_base.hpp"

namespace micros_rtt
{
template <class M>
class InterPublication : public ConnectionBase
{
public:
  InterPublication(const std::string& topic) : ConnectionBase(topic, true) {}
  ~InterPublication() {}

  bool publish(M message)
  {
    typename ChannelElement<M>::shared_ptr output
                = boost::static_pointer_cast< ChannelElement<M> >(this->getChannelElement());
    if (output)
    {
      output->write(message);
      return true;
    }
    else
    {
      return false;
    }
  }
};

}

#endif
