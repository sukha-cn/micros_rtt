#ifndef HPCLRTT_CONN_OUTPUT_ENDPOINT_HPP
#define HPCLRTT_CONN_OUTPUT_ENDPOINT_HPP

#include "channel_data_element.hpp"
#include "../subscription.h"

namespace hpcl_rtt
{
/** This is a channel element that represents the output endpoint of a
 * connection, i.e. the part that is connected to the InputPort.
 * In the RTT, connections are always created from input towards output.
 * So this class is typically first created of the channel element chain
 * and attached to the input port. Then we build further towards the
 * outputport. Imagine a spider attaching a thread at one wall and
 * moving along to the other side of the wall.
 */
template<typename T>
class ConnOutputEndpoint : public ChannelElement<T>
{
  ConnectionBasePtr port;
public:
  /**
   * Creates the connection end that represents the output and attach
   * it to the input.
   * @param port The start point.
   * @param output_id Each connection must be identified by an ID that
   * represents the other end. This id is passed to the input port \a port.
   * @return
   */
  ConnOutputEndpoint(ConnectionBasePtr port)
        : port(port)
  {
    // cid is deleted/owned by the port's ConnectionManager.
  }

  ~ConnOutputEndpoint()
  {
  }

  /** Called by the connection factory to check that the connection is
   * properly set up. It is called when the channel is complete, so we can
   * register ourselves on the port side now
   *
   * Before that, the channel might not be complete and therefore having
   * the input port read on it would lead to crashes
   */
  bool inputReady()
  {
    // cid is deleted/owned by the ConnectionManager.
    port->addConnection(this);
    return ChannelElement<T>::inputReady();
  }

  using ChannelElement<T>::write;

  /** Writes a new sample on this connection
   * This should never be called, as all connections are supposed to have
   * a data storage element */
  virtual bool write(typename ChannelElement<T>::param_t sample)
  { return false; }

  virtual void disconnect(bool forward)
  {
//    // Call the base class: it does the common cleanup
//    base::ChannelElement<T>::disconnect(forward);
//
//    InputPort<T>* port = this->port;
//    if (port && forward)
//    {
//      this->port = 0;
//      port->removeConnection(cid);
//    }
  }

  virtual bool signal()
  {
//    InputPort<T>* port = this->port;
//#ifdef ORO_SIGNALLING_PORTS
//    if (port && port->new_data_on_port_event)
//      (*port->new_data_on_port_event)(port);
//#else
//    if (port )
//      port->signal();
//#endif
    return true;
  }

  virtual bool data_sample(typename ChannelElement<T>::param_t sample)
  {
    return true;
  }

};

}

#endif
