/* 
 *  conn_output_endpoint.hpp - micros output end element for transport channel
 *  Copyright (C) 2015 Zaile Jiang
 *  
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/
#ifndef MICROSRTT_CONN_OUTPUT_ENDPOINT_HPP
#define MICROSRTT_CONN_OUTPUT_ENDPOINT_HPP

#include "micros_rtt/connection_base.h"
#include "micros_rtt/oro/channel_data_element.hpp"

namespace micros_rtt
{
/** This is a channel element that represents the output endpoint of a
 * connection, i.e. the part that is connected to the InputPort.
 * In the RTT, connections are always created from input towards output.
 * So this class is typically first created of the channel element chain
 * and attached to the input port. Then we build further towards the
 * outputport. Imagine a spider attaching a thread at one wall and
 * moving along to the other side of the wall.
 */
template<typename M>
class ConnOutputEndpoint : public ChannelElement<M>
{
  ConnectionBasePtr subscription_;
public:
  /**
   * Creates the connection end that represents the output and attach
   * it to the input.
   * @param port The start point.
   * @param output_id Each connection must be identified by an ID that
   * represents the other end. This id is passed to the input port \a port.
   * @return
   */
  ConnOutputEndpoint(ConnectionBasePtr subscription)
        : subscription_(subscription)
  {
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
    return ChannelElement<M>::inputReady();
  }

  using ChannelElement<M>::write;

  /** Writes a new sample on this connection
   * This should never be called, as all connections are supposed to have
   * a data storage element */
  virtual bool write(typename ChannelElement<M>::param_t sample)
  { return false; }

  virtual void disconnect(bool forward)
  {
    //temporary not supportted
  }

  virtual bool signal()
  {
    //temporary not supportted
    return true;
  }

  virtual bool data_sample(typename ChannelElement<M>::param_t sample)
  {
    //data element has been initialized, this should be true
    return true;
  }

};

}

#endif
