#include "oro/connection_factory.hpp"

namespace micros_rtt
{
  
static bool ConnFactory::createAndCheckConnection(ConnectionBasePtr publication, 
          ConnectionBasePtr subscription, ChannelElementBase::shared_ptr channel_input)
{
  // Register the subscription to the publication.
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
  return false;
}

bool ConnFactory::createAndCheckStream(base::OutputPortInterface& output_port, ConnPolicy const& policy, base::ChannelElementBase::shared_ptr chan, StreamConnID* conn_id) {
    if (policy.transport == 0 ) {
        log(Error) << "Need a transport for creating streams." <<endlog();
        return false;
    }
    const types::TypeInfo* type = output_port.getTypeInfo();
    if ( type->getProtocol(policy.transport) == 0 ) {
        log(Error) << "Could not create transport stream for port "<< output_port.getName() << " with transport id " << policy.transport <<endlog();
        log(Error) << "No such transport registered. Check your policy.transport settings or add the transport for type "<< type->getTypeName() <<endlog();
        return false;
    }
    types::TypeMarshaller* ttt = dynamic_cast<types::TypeMarshaller*> ( type->getProtocol(policy.transport) );
    if (ttt) {
        int size_hint = ttt->getSampleSize( output_port.getDataSource() );
        policy.data_size = size_hint;
    } else {
        log(Debug) <<"Could not determine sample size for type " << type->getTypeName() << endlog();
    }
    RTT::base::ChannelElementBase::shared_ptr chan_stream = type->getProtocol(policy.transport)->createStream(&output_port, policy, true);
            
    if ( !chan_stream ) {
        log(Error) << "Transport failed to create remote channel for output stream of port "<<output_port.getName() << endlog();
        return false;
    }
    chan->setOutput( chan_stream );

    if ( output_port.addConnection( new StreamConnID(policy.name_id), chan, policy) ) {
        log(Info) << "Created output stream for output port "<< output_port.getName() <<endlog();
        return true;
    }
    // setup failed.
    log(Error) << "Failed to create output stream for output port "<< output_port.getName() <<endlog();
    return false;
}


}
