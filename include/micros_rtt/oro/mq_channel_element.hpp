/* 
 *  mq_channel_element.hpp - micros message queue element for transport channel
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
#ifndef MICROSRTT_MQ_CHANNEL_ELEMENT_HPP
#define MICROSRTT_MQ_CHANNEL_ELEMENT_HPP

#include "ros/serialization.h"
#include "micros_rtt/oro/mq_send_recv.hpp"
#include "micros_rtt/oro/channel_element.hpp"
#include "micros_rtt/oro/data_lockfree.hpp"
#include <stdexcept>

namespace micros_rtt
{
using namespace ros::serialization;
/**
 * Implements the a ChannelElement using message queues.
 * It converts the C++ calls into MQ messages and vice versa.
 * @todo This class can be refactored into a base class with
 * generic mqueue code and a subclass with type specific info.
 * @todo This is an inspiration for a generic, transport independent
 * channel element.
 */
template<typename M>
class MQChannelElement: public ChannelElement<M>, public MQSendRecv
{
  /** Used as a temporary on the reading side */
  typename DataObjectLockFree<M>::shared_ptr read_sample;
  /** Used in write() to refer to the sample that needs to be written */
  typename DataObjectLockFree<M>::shared_ptr write_sample;

  
  SerializedMessage read_m;
  SerializedMessage write_m;

public:
  /**
   * Create a channel element for remote data exchange.
   * @param transport The type specific object that will be used to marshal the data.
   */
  MQChannelElement(ConnectionBasePtr connection, bool is_sender)
      : MQSendRecv(), 
        read_sample(new DataObjectLockFree<M>(M()))
        , write_sample(new DataObjectLockFree<M>(M()))

  {
    setupStream(connection, sizeof(M), is_sender);
  }

  ~MQChannelElement() 
  {
    cleanupStream();
  }

  virtual bool inputReady() 
  {
    if ( mqReady(this) ) 
    {
//      typename ChannelElement<M>::shared_ptr output = this->getOutput();
//      assert(output);
//      output->data_sample(read_sample->Get());
      return true;
    }
    return false;
  }

  virtual bool data_sample(typename ChannelElement<M>::param_t sample)
  {
    // send initial data sample to the other side using a plain write.
    if (mis_sender) 
    {
      typename ChannelElement<M>::shared_ptr output =
          this->getOutput();
 
      write_sample->data_sample(sample);
      // update MQSendRecv buffer:
      // mqNewSample(sizeof(sample));
      // return mqWrite(sample);
      
      return true;
    }
    return false;
  }

  /**
   * Signal will cause a read-write cycle to transfer the
   * data from the data/buffer element to the message queue
   * and vice versa.
   *
   * Note: this virtual function is a bit abused. For a sending
   * MQ, signal triggers a direct read on the data element.
   * For a receiving MQ, signal is used by the dispatcher thread
   * to provoque a read from the MQ and forward it to the next
   * channel element.
   *
   * In the sending case, signal could trigger a dispatcher thread
   * that does the read/write cycle, but that seems only causing overhead.
   * The receiving case must use a thread which blocks on all mq
   * file descriptors.
   * @return true in case the forwarding could be done, false otherwise.
   */
//  bool signal()
//  {
//    // copy messages into channel
//    if (mis_sender) 
//    {
//      // this read should always succeed since signal() means
//      // 'data available in a data element'.
//      typename ChannelElement<T>::shared_ptr input =
//          this->getInput();
//      if( input && input->read(read_sample->set(), false) == NewData )
//        return this->write(read_sample->rvalue());
//    } 
//    else 
//    {
//      typename ChannelElement<T>::shared_ptr output =
//          this->getOutput();
//      if (output && mqRead())
//        return output->write(read_sample->rvalue());
//      ;
//    }
//    return false;
//  }

  /**
   * Read from the message queue.
   * @param sample stores the resulting data sample.
   * @return true if an item could be read.
   */
  FlowStatus read(typename ChannelElement<M>::reference_t sample, bool copy_old_data)
  {
    ROS_DEBUG("micros message queue read.");
    //messages got from message queue need to be deserialize.
        
    if (mqRead(read_m))
    {
      deserializeMessage<M>(read_m, sample);
      return NewData;
    }
    
    return NoData;
  }

  /**
   * Write to the message queue
   * @param sample the data sample to write
   * @return true if it could be sent.
   */
  bool write(typename ChannelElement<M>::param_t sample)
  {
    //messages sent through message queue need to be serialized first.
    uint32_t len = serializationLength(sample);
    write_m.num_bytes = len + 4;
    if (write_m.buf == NULL)
      write_m.buf.reset(new uint8_t[write_m.num_bytes]);
  
    OStream s(write_m.buf.get(), (uint32_t)write_m.num_bytes);
    serialize(s, (uint32_t)write_m.num_bytes - 4);
    write_m.message_start = s.getData();
    serialize(s, sample);

    return mqWrite(write_m);
  }
};

}

#endif

