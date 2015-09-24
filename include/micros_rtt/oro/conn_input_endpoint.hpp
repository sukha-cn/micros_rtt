/* 
 *  conn_input_endpoint.hpp - micros input end element for transport channel
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
#ifndef MICROSRTT_CONN_INPUT_ENDPOINT_HPP
#define MICROSRTT_CONN_INPUT_ENDPOINT_HPP

#include "micros_rtt/connection_base.h"
#include "micros_rtt/oro/channel_data_element.hpp"

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
  virtual FlowStatus read(typename ChannelElement<M>::reference_t sample)
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
