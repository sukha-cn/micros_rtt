#include <fcntl.h>
#include <sys/stat.h>
#include <mqueue.h>
#include <sys/types.h>
#include <unistd.h>
#include <sstream>
#include <cassert>
#include <stdexcept>
#include <errno.h>

#include "micros_rtt/common.h"
#include "micros_rtt/oro/mqueue/MQSendRecv.hpp"
#include "std_msgs/String.h"


namespace micros_rtt
{


MQSendRecv::MQSendRecv() : buf(0), 
  mis_sender(false), minit_done(false), max_size(0), mdata_size(0)
{
}

void MQSendRecv::setupStream(ConnectionBasePtr connection, int data_size,
                             bool is_sender)
{
  mdata_size = data_size;
  max_size = data_size * 10;
  mis_sender = is_sender;

  std::stringstream namestr;
  namestr << '/' << connection->getTopic();

  struct mq_attr mattr;
  mattr.mq_maxmsg = data_size;
  mattr.mq_msgsize = max_size;
  assert( max_size );
  if (max_size <= 0)
    throw std::runtime_error("Could not open message queue with zero message size.");


  // set message queue flag and create stream
  int oflag = O_CREAT;
  if (mis_sender)
      oflag |= O_WRONLY | O_NONBLOCK;
  else
      oflag |= O_RDONLY | O_NONBLOCK;
  mqdes = mq_open(namestr.str().c_str(), oflag, S_IREAD | S_IWRITE, &mattr);

  if (mqdes < 0)
  {
    int the_error = errno;
    ROS_WARN("micros FAILED opening message queue %s with message size %d, buffer size %d.", namestr.str().c_str(), (int)mattr.mq_msgsize, (int)mattr.mq_maxmsg);
    // these are copied from the man page. They are more informative than the plain perrno() text.
    switch (the_error)
    {
    case EACCES:
      ROS_WARN("The queue exists, but the caller does not have permission to open it in the specified mode.");
      break;
    case EINVAL:
        // or the name is wrong...
      ROS_WARN("Wrong mqueue name given OR, In a process  that  is  unprivileged (does  not  have  the CAP_SYS_RESOURCE  capability),  attr->mq_maxmsg must be less than or equal to the msg_max limit, and attr->mq_msgsize must be less than or equal to the msgsize_max limit.  In addition, even in a privileged process, attr->mq_maxmsg cannot exceed the HARD_MAX limit. (See mq_overview(7) for details of these limits.");
      break;
    case EMFILE:
      ROS_WARN("The process already has the maximum number of files and message queues open.");
      break;
    case ENAMETOOLONG:
      ROS_WARN("Name was too long.");
      break;
    case ENFILE:
      ROS_WARN("The system limit on the total number of open files and message queues has been reached.");
      break;
    case ENOSPC:
      ROS_WARN( "Insufficient space for the creation of a new message queue. This probably occurred because the queues_max limit was encountered; see mq_overview(7).");
      break;
    case ENOMEM:
      ROS_WARN("Insufficient memory.");
      break;
    default:
      ROS_WARN("Submit a bug report. An unexpected mq error occured with errno= %x : %s", errno, strerror(errno));
    }
    throw std::runtime_error("Could not open message queue: mq_open returned -1.");
  }

  buf = new char[max_size];
  memset(buf, 0, max_size); // necessary to trick valgrind
  mqname = namestr.str();
  
  ROS_DEBUG("micros open Mqueue with name %s", mqname.c_str());
}

MQSendRecv::~MQSendRecv()
{
  if (mqdes > 0)
    mq_close(mqdes);
}


void MQSendRecv::cleanupStream()
{
  ROS_DEBUG("cleanupStream");
  if (!mis_sender)
  {
    if (minit_done)
    {
      minit_done = false;
    }
  }
  else
  {
    // sender unlinks to avoid future re-use of new readers.
    mq_unlink(mqname.c_str());
  }
  // both sender and receiver close their end.
  mq_close( mqdes);

  if (buf)
  {
    delete[] buf;
    buf = 0;
  }
}


void MQSendRecv::mqNewSample(int size)
{
    // only deduce if user did not specify it explicitly:
    if (mdata_size == 0)
      max_size = size;
    delete[] buf;
    buf = new char[max_size];
    memset(buf, 0, max_size); // necessary to trick valgrind
}

bool MQSendRecv::mqReady(ChannelElementBase* chan)
{
  if (minit_done)
    return true;

  if (!mis_sender)
  {
    // Try to get the initial sample
    // The output port implementation guarantees that there will be one
    // after the connection is ready
    ROS_DEBUG("micros message queue ready.");
    return true;
  }
  else
  {
    assert( !mis_sender ); // we must be receiver. we can only receive inputReady when we're on the input port side of the MQ.
    return false;
  }
}

bool MQSendRecv::mqRead(SerializedMessage& m)
{
  int bytes = 0;
  if ((bytes = mq_receive(mqdes, buf, max_size, 0)) == -1)
  {
    ROS_WARN("micros message queue tried to read on empty mq!");
    return false;
  }
  else
  {
    ROS_DEBUG("micros message queue received %d bytes.", bytes);
    if (!m.buf)
      m.buf.reset((unsigned char *)buf);
    m.num_bytes = bytes;
    m.message_start = ((unsigned char *)buf + 4);
    return true;
  }
}

bool MQSendRecv::mqWrite(SerializedMessage& m)
{
  if (!m.buf.get())
	  ROS_WARN("micros message queue write null buf");
  if (mq_send(mqdes, (char *)m.buf.get(), (uint32_t)m.num_bytes, 0) == -1)
  {
    //ROS_WARN("micros message queue send error number:%x.", errno);
    if (errno == EAGAIN)
    {
			//ROS_WARN("EAGAIN, message queue full.");
      return true;
    }
    return false;
  }
	ROS_DEBUG("micros message queue write successfully.");
  return true;
}

}
