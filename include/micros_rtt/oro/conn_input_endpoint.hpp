#ifndef MICROSRTT_CONN_INPUT_ENDPOINT_HPP
#define MICROSRTT_CONN_INPUT_ENDPOINT_HPP

#include "channel_data_element.hpp"
#include "../connection_base.hpp"

namespace micros_rtt
{
/** This is a channel element that represents the input endpoint of a
 * connection, i.e. the part that is connected to the OutputPort
 */
template<typename M>
class ConnInputEndpoint : public ChannelElement<M>
{
  ConnectionBasePtr publication_;

public:
  ConnInputEndpoint(ConnectionBasePtr publication) : publication_(publication) { }

  ~ConnInputEndpoint()
  {
  }

  using ChannelElement<M>::read;

  /** Reads a new sample from this connection
   * This should never be called, as all connections are supposed to have
   * a data storage element */
  virtual FlowStatus read(typename ChannelElement<T>::reference_t sample)
  { return NoData; }

  virtual bool inputReady() 
  {
    return true;
  }

  virtual void disconnect(bool forward)
  {
    //temporary not supportted
  }
};

}

#endif
