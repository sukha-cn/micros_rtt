#include "micros_rtt/oro/connection_factory.hpp"

namespace micros_rtt
{
  
bool ConnFactory::createAndCheckConnection(ConnectionBasePtr publication, 
          ConnectionBasePtr subscription, ChannelElementBase::shared_ptr channel_input)
{
  // Register the subscription to the publication.
  if ( publication->addConnection(channel_input) ) 
  {
    ROS_DEBUG("micros connection factory adds input_end of channel to publication:%s.", publication->getTopic().c_str());
    // notify input that the connection is now complete.
    if ( subscription->channelReady( channel_input->getOutputEndPoint() ) == false ) 
    {
      ROS_WARN("micros connection factory checks channel ready failed, subscription:%s", subscription->getTopic().c_str());
      return false;
    }
    return true;
  }
  // setup failed.
  return false;
}

bool ConnFactory::createAndCheckStream(ConnectionBasePtr connection, 
        ChannelElementBase::shared_ptr chan, bool is_sender)
{
  if (is_sender)
  {
    if ( connection->addMQConnection(chan) ) 
    {
      ROS_DEBUG("micros connection factory adds input_end of stream to publication:%s.", connection->getTopic().c_str());
      return true;
    }
    // setup failed.
    ROS_WARN("micros connection factory fail to add input_end of stream to publication:%s.", connection->getTopic().c_str());
    return false;
  }
  else
  {
    if ( connection->channelReady( chan->getOutputEndPoint() ) == true ) 
    {
      ROS_DEBUG("micros connection factory adds output_end of stream to subscription:%s.", connection->getTopic().c_str());
      return true;
    }
    // setup failed: manual cleanup.
    chan = 0; 
    ROS_WARN("micros connection factory fail to add output_end of stream to subscription:%s.", connection->getTopic().c_str());
    return false;
  }
}

}
