#ifndef MICROSRTT_INTER_PUBLICATION_H
#define MICROSRTT_INTER_PUBLICATION_H

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
  }
};

}

#endif
