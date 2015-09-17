#ifndef MICROSRTT_MQSENDER_HPP_
#define MICROSRTT_MQSENDER_HPP_

#include <ros/ros.h>
#include <mqueue.h>
#include "ros/serialization.h"
#include "micros_rtt/connection_base.hpp"
#include "micros_rtt/oro/channel_element.hpp"

namespace micros_rtt
{
using namespace ros::serialization;
using ros::SerializedMessage;
/**
 * Implements the sending/receiving of mqueue messages.
 * It can only be OR sender OR receiver (logical XOR).
 */
class MQSendRecv
{
protected:

  /**
   * MQueue file descriptor.
   */
  mqd_t mqdes;
  /**
   * Send/Receive buffer. It is initialized to the size of the value
   * provided by the ConnPolicy or, if the policy has a zero data
   * size, the sample given to setupStream
   *
   * Its size is saved in max_size
   */
  char* buf;
  /**
   * True if this object is a sender.
   */
  bool mis_sender;
  /**
   * True if setupStream() was called, false after cleanupStream().
   */
  bool minit_done;
  /**
   * The size of buf.
   */
  int max_size;
  /**
   * The name of the queue, as specified in the ConnPolicy when
   * creating the stream, or self-calculated when that name was empty.
   */
  std::string mqname;
  /**
   * The size of the data, as specified in the ConnPolicy when
   * creating the stream, or calculated using the transport when
   * that size was zero.
   */
  int mdata_size;

public:
  /**
   * Create a channel element for remote data exchange.
   * @param transport The type specific object that will be used to marshal the data.
   */
  MQSendRecv();

  void setupStream(ConnectionBasePtr connection, int size, bool is_sender);

  ~MQSendRecv();

  void cleanupStream();

  /**
   * Adapts the mq send/receive buffer size according to the
   * data in mqdata_source, or the value set in mdata_size;
   * @param sample
   */
  virtual void mqNewSample(int size);

  /**
   * Works only in receive mode, waits for a new sample and
   * adapts the receive buffer to match it's size.
   * @return
   */
  virtual bool mqReady(ChannelElementBase* chan);

  /**
   * Read from the message queue.
   * @param sample stores the resulting data sample.
   * @return true if an item could be read.
   */
  bool mqRead(SerializedMessage& m);

  /**
   * Write to the message queue
   * @param ds the data sample to write
   * @param is_data_sample true if the sample is used for initialization, false if it is a proper write
   * @return true if it could be sent.
   */
  bool mqWrite(SerializedMessage m);
};
}

#endif /* ORO_MQSENDER_HPP_ */
