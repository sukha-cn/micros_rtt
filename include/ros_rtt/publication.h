#ifndef ROSRTT_PUBLICATION_H
#define ROSRTT_PUBLICATION_H

#include "oro/channel_data_element.hpp"
#include "ros/ros.h"
#include "ros_rtt/connection_base.h"
namespace rosrtt
{
//class Publication;
//typedef boost::shared_ptr<Publication> PublicationPtr;
//typedef std::vector<PublicationPtr> V_Publication;

template <class M>
class Publication : public ConnectionBase
{
public:
  typedef boost::shared_ptr< Publication<M> > shared_ptr;
  
  Publication(const std::string& topic) : ConnectionBase(topic) {}
  ~Publication() {}

  bool publish(M message)
  {
    typename ChannelElement<M>::shared_ptr output
                = boost::static_pointer_cast< ChannelElement<M> >(this->getChannelElement());
    if (output)
    {
      output->write(message);
      return true;
    }
    else
    {
      return false;
    }
  }

};

}

#endif
