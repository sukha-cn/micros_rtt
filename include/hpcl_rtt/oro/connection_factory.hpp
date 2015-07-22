#ifndef ROSRTT_CONN_FACTORY_HPP
#define ROSRTT_CONN_FACTORY_HPP

#include "channel_element.hpp"
#include "conn_input_endpoint.hpp"
#include "conn_output_endpoint.hpp"
#include "data_lockfree.hpp"

namespace hpcl_rtt
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
};

}
#endif

