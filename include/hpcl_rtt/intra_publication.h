#ifndef HPCLRTT_INTRA_PUBLICATION_H
#define HPCLRTT_INTRA_PUBLICATION_H

#include "ros/ros.h"
#include "hpcl_rtt/connection_base.h"

namespace hpcl_rtt
{
template <class M>
class IntraPublication : public ConnectionBase
{
public:
  IntraPublication(const std::string& topic) : ConnectionBase(topic, true) {}
  ~IntraPublication() {}

  bool publish(M message)
  {
  }
}

}

#endif
