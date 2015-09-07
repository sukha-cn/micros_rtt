#ifndef MICROSRTT_MQ_CHANNEL_ELEMENT_H
#define MICROSRTT_MQ_CHANNEL_ELEMENT_H

#include "MQSendRecv.hpp"
#include "../channel_element.hpp"
#include <stdexcept>

namespace micros_rtt
{
  /**
   * Implements the a ChannelElement using message queues.
   * It converts the C++ calls into MQ messages and vice versa.
   * @todo This class can be refactored into a base class with
   * generic mqueue code and a subclass with type specific info.
   * @todo This is an inspiration for a generic, transport independent
   * channel element.
   */
  template<typename T>
  class MQChannelElement: public ChannelElement<T>, public MQSendRecv
  {
    /** Used as a temporary on the reading side */
//    typename ValueDataSource<T>::shared_ptr read_sample;
    /** Used in write() to refer to the sample that needs to be written */
//    typename LateConstReferenceDataSource<T>::shared_ptr write_sample;

  public:
       /**
        * Create a channel element for remote data exchange.
        * @param transport The type specific object that will be used to marshal the data.
        */
       MQChannelElement(ConnectionBasePtr connection, bool is_sender)
           : MQSendRecv()
//           , read_sample(new ValueDataSource<T>)
//           , write_sample(new LateConstReferenceDataSource<T>)

       {
           setupStream(connection, sizeof(T), is_sender);
       }

       ~MQChannelElement() {
           cleanupStream();
       }

       virtual bool inputReady() {
           if ( mqReady(this) ) {
               typename ChannelElement<T>::shared_ptr output =
                   this->getOutput();
               assert(output);
 //              output->data_sample(read_sample->rvalue());
               return true;
           }
           return false;
       }

       virtual bool data_sample(typename ChannelElement<T>::param_t sample)
       {
           // send initial data sample to the other side using a plain write.
           if (mis_sender) {
               typename ChannelElement<T>::shared_ptr output =
                   this->getOutput();

  //             write_sample->setPointer(&sample);
               // update MQSendRecv buffer:
  //             mqNewSample(write_sample);
  //             return mqWrite(write_sample);
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
       bool signal()
       {
           // copy messages into channel
           if (mis_sender) {
               // this read should always succeed since signal() means
               // 'data available in a data element'.
               typename ChannelElement<T>::shared_ptr input =
                   this->getInput();
//               if( input && input->read(read_sample->set(), false) == NewData )
//                   return this->write(read_sample->rvalue());
           } else {
               typename ChannelElement<T>::shared_ptr output =
                   this->getOutput();
               if (output && mqRead())
//                   return output->write(read_sample->rvalue());
               ;
           }
           return false;
       }

       /**
        * Read from the message queue.
        * @param sample stores the resulting data sample.
        * @return true if an item could be read.
        */
       FlowStatus read(typename ChannelElement<T>::reference_t sample, bool copy_old_data)
       {
           throw std::runtime_error("not implemented");
       }

       /**
        * Write to the message queue
        * @param sample the data sample to write
        * @return true if it could be sent.
        */
       bool write(typename ChannelElement<T>::param_t sample)
       {
//           write_sample->setPointer(&sample);
           return mqWrite();
       }

   };
 
}

#endif

