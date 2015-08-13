#ifndef HPCLRTT_CONN_FACTORY_HPP
#define HPCLRTT_CONN_FACTORY_HPP

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

  template<class T>
  static bool createStream(IntraPublication <T>& publication, std::string &topic)
  {
    ChannelElementBase::shared_ptr chan = buildChannelInput(publication, base::ChannelElementBase::shared_ptr() );
    return createAndCheckStream(publication, chan);
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
  static bool createAndCheckStream(IntraPublication <T>& publication, base::ChannelElementBase::shared_ptr chan) 
  {
    RTT::base::ChannelElementBase::shared_ptr chan_stream = createMqStream(publication, true);
              
    if ( !chan_stream ) {
      ROSINFO("Transport failed to create remote channel for output stream of port ");
      return false;
    }
    chan->setOutput( chan_stream );
  
    if ( output_port.addConnection( new StreamConnID(policy.name_id), chan, policy) ) {
      ROSINFO("Created output stream for output port ");
      return true;
    }
    // setup failed.
    ROSINRO("Failed to create output stream for output port ");
    return false;
  }  

  template<typename T>
  ChannelElementBase::shared_ptr createMqStream(IntraPublication <T>& publication, bool is_sender) const
  {
    try
    {
      ChannelElementBase::shared_ptr mq = new MQChannelElement<T>(port, *this, is_sender);
      if (!is_sender)
      {
      // the receiver needs a buffer to store his messages in.
        ChannelElementBase::shared_ptr buf = buildDataStorage();
        mq->setOutput(buf);
      }
      return mq;
    }
    catch(std::exception& e)
    {
      ROSINFO("Failed to create MQueue Channel element: ");
    }
    return ChannelElementBase::shared_ptr();
  } 

};

}
#endif

