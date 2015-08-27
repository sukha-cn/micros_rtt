#ifndef HPCLRTT_INTER_PUBLICATION_H
#define HPCLRTT_INTER_PUBLICATION_H

#include "ros/ros.h"
#include "hpcl_rtt/connection_base.h"

namespace hpcl_rtt
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
