#ifndef MICROSRTT_CONN_FACTORY_HPP
#define MICROSRTT_CONN_FACTORY_HPP

#include "ros/ros.h"
#include "channel_element.hpp"
#include "mqueue/MQChannelElement.hpp"
#include "../connection_base.hpp"
#include "conn_input_endpoint.hpp"
#include "conn_output_endpoint.hpp"
#include "data_lockfree.hpp"
#include "../inter_publication.h"
#include "../inter_subscription.h"

namespace micros_rtt
{

/** This class provides the basic tools to create channels that represent
 * connections between two ports.
 *
 * The ports and type transports use these functions to setup connections.
 * The interface may change as the needs of these 'users' change.
 */
class ConnFactory
{
public:
  ConnFactory() {}
  ~ConnFactory() {}

  template<typename T>
  static ChannelElementBase* buildDataStorage(const T& initial_value = T())
  {
    typename DataObjectLockFree<T>::shared_ptr data_object;
    data_object.reset( new DataObjectLockFree<T>(initial_value) );
    ChannelDataElement<T>* result = new ChannelDataElement<T>(data_object);
    return result;
  }

  template<typename T>
  static ChannelElementBase::shared_ptr buildChannelInput(ConnectionBasePtr publication, ChannelElementBase::shared_ptr output_channel)
  {
    ChannelElementBase::shared_ptr endpoint = new ConnInputEndpoint<T>(publication);
    if (output_channel)
      endpoint->setOutput(output_channel);
    return endpoint;
  }

  template<typename T>
  static ChannelElementBase::shared_ptr buildChannelOutput(ConnectionBasePtr subscription)
  {
    ChannelElementBase::shared_ptr endpoint = new ConnOutputEndpoint<T>(subscription);
    return endpoint;
  }
  
  template<typename T>
  static ChannelElementBase::shared_ptr buildBufferedChannelOutput(ConnectionBasePtr subscription, T const& initial_value = T() )
  {
    ChannelElementBase::shared_ptr endpoint = new ConnOutputEndpoint<T>(subscription);
    ChannelElementBase::shared_ptr data_object = buildDataStorage<T>(initial_value);
    data_object->setOutput(endpoint);
    return data_object;
  }

  template<typename T>
  static bool createConnection(ConnectionBasePtr publication, ConnectionBasePtr subscription)
  {
    // This is the input channel element of the output half
    ChannelElementBase::shared_ptr output_half = 0;
    // local ports, create buffer here.
    output_half = buildBufferedChannelOutput<T>(subscription);

    if (!output_half)
      return false;

    // Since output is local, buildChannelInput is local as well.
    // This this the input channel element of the whole connection
    ChannelElementBase::shared_ptr channel_input =
      buildChannelInput<T>(publication, output_half);

    return createAndCheckConnection(publication, subscription, channel_input);
  }

  template<class T>
  static bool createStream(ConnectionBasePtr connection, const std::string &topic, bool is_sender)
  {
    ChannelElementBase::shared_ptr chan;
    if (is_sender)
    {
      ROS_INFO("connection factory creating publication stream.");
      chan = buildChannelInput<T>(connection, ChannelElementBase::shared_ptr() );
    }
    else
    {
      ROS_INFO("connection factory creating subscription stream.");
      chan = buildChannelOutput<T>(connection);
    }
    return createAndCheckStream<T>(connection, chan, is_sender);
  }

protected:
  //template<typename T>
  static bool createAndCheckConnection(ConnectionBasePtr publication, ConnectionBasePtr subscription, ChannelElementBase::shared_ptr channel_input)
  {
    // Register the channel's input to the output port.
    if ( publication->addConnection(channel_input) ) 
    {
      // notify input that the connection is now complete.
      
      if ( channel_input->getOutputEndPoint()->inputReady() == false ) 
      {
        return false;
      }
      return true;
    }
    // setup failed.
    channel_input->disconnect(true);
    return false;
  }

  template<typename T>
  static bool createAndCheckStream(ConnectionBasePtr connection, ChannelElementBase::shared_ptr chan, bool is_sender) 
  {
    if (is_sender)
    {
      ChannelElementBase::shared_ptr chan_stream = createMqStream<T>(connection, true);
                
      if ( !chan_stream ) {
        ROS_INFO("Transport failed to create remote channel for output stream of port ");
        return false;
      }
      chan->setOutput( chan_stream );
    
      if ( connection->addConnection(chan) ) {
        ROS_INFO("Created output stream for output port ");
        return true;
      }
      // setup failed.
      ROS_INFO("Failed to create output stream for output port ");
      return false;
    }
    else
    {
      // note: don't refcount this final input chan, because no one will
      // take a reference to it. It would be destroyed upon return of this function.
      ChannelElementBase::shared_ptr chan_stream = createMqStream<T>(connection, false);
      
      if ( !chan_stream ) {
          ROS_INFO("Transport failed to create remote channel for input stream of port ");
          return false;
      }
    
      chan_stream->getOutputEndPoint()->setOutput( chan );
      if ( connection->channelReady( chan_stream->getOutputEndPoint() ) == true ) {
          ROS_INFO("Created input stream for input port ");
          return true;
      }
      // setup failed: manual cleanup.
      chan_stream = 0; // deleted by channelReady() above !
      ROS_INFO("Failed to create input stream for input port ");
      return false;
    }
  }  
  

  template<typename T>
  static ChannelElementBase::shared_ptr createMqStream(ConnectionBasePtr connection, bool is_sender) 
  {
    try
    {
      ChannelElementBase::shared_ptr mq = new MQChannelElement<T>(connection, is_sender);
      if (!is_sender)
      {
        // the receiver needs a buffer to store his messages in.
        ChannelElementBase::shared_ptr buf = buildDataStorage<T>();
        mq->setOutput(buf);
      }
      return mq;
    }
    catch(std::exception& e)
    {
      ROS_INFO("Failed to create MQueue Channel element: ");
    }
    return ChannelElementBase::shared_ptr();
  } 

};

}
#endif

