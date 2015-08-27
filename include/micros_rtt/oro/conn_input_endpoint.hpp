#ifndef HPCLRTT_CONN_INPUT_ENDPOINT_HPP
#define HPCLRTT_CONN_INPUT_ENDPOINT_HPP

#include "channel_data_element.hpp"
#include "../connection_base.h"

namespace hpcl_rtt
{
/** This is a channel element that represents the input endpoint of a
 * connection, i.e. the part that is connected to the OutputPort
 */
template<typename T>
class ConnInputEndpoint : public ChannelElement<T>
{
  ConnectionBasePtr port;

public:
  ConnInputEndpoint(ConnectionBasePtr port) : port(port) { }

  ~ConnInputEndpoint()
  {
    //this->disconnect(false); // inform port (if any) we're gone.
  }

  using ChannelElement<T>::read;

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
      // Call the base class first
//      base::ChannelElement<T>::disconnect(forward);
//
//      Publication<T>* port = this->port;
//      if (port && !forward)
//      {
//        this->port   = 0;
//        port->removeConnection();
//      }
  }
};

}

#endif
