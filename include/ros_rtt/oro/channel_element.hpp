#ifndef ROSRTT_CHANNEL_ELEMENT_HPP
#define ROSRTT_CHANNEL_ELEMENT_HPP

#include <boost/intrusive_ptr.hpp>
#include <boost/call_traits.hpp>
#include "channel_element_base.hpp"
#include "../common.h"

namespace rosrtt 
{   

/** A typed version of ChannelElementBase. It defines generic methods that are
 * type-specific (like write and read)
 */
template<typename T>
class ChannelElement : public ChannelElementBase
{
public:
  typedef T value_t;
  typedef boost::intrusive_ptr< ChannelElement<T> > shared_ptr;
  typedef typename boost::call_traits<T>::param_type param_t;
  typedef typename boost::call_traits<T>::reference reference_t;

  shared_ptr getOutput()
  {
    return boost::static_pointer_cast< ChannelElement<T> >(ChannelElementBase::getOutput());
  }

  shared_ptr getInput()
  {
    return boost::static_pointer_cast< ChannelElement<T> >(ChannelElementBase::getInput());
  }

  /**
   * Provides a data sample to initialize this connection.
   * This is used before the first write() in order to inform this
   * connection of the size of the data. As such enough storage
   * space can be allocated before the actual writing begins.
   *
   * @returns false if an error occured that requires the channel to be invalidated.
   */
  virtual bool data_sample(param_t sample)
  {
    typename ChannelElement<T>::shared_ptr output = boost::static_pointer_cast< ChannelElement<T> >(getOutput());
    if (output)
    {
      return output->data_sample(sample);
    }
    return false;
  }

  virtual value_t data_sample()
  {
    typename ChannelElement<T>::shared_ptr input = boost::static_pointer_cast< ChannelElement<T> >(getInput());
    if (input)
    {
      return input->data_sample();
    }
    return value_t();
  }

  /** Writes a new sample on this connection. \a sample is the sample to
   * write. 
   *
   * @returns false if an error occured that requires the channel to be invalidated. In no ways it indicates that the sample has been received by the other side of the channel.
   */
  virtual bool write(param_t sample)
  {
    typename ChannelElement<T>::shared_ptr output = boost::static_pointer_cast< ChannelElement<T> >(getOutput());
    if (output)
    {
      return output->write(sample);
    }
    return false;
  }

  /** Reads a sample from the connection. \a sample is a reference which
   * will get updated if a sample is available. The method returns true
   * if a sample was available, and false otherwise. If false is returned,
   * then \a sample is not modified by the method
   */
  virtual FlowStatus read(reference_t sample, bool copy_old_data)
  {
    typename ChannelElement<T>::shared_ptr input = this->getInput();
    if (input)
    {
      return input->read(sample, copy_old_data);
    }
    else
    {
      return NoData;
    }
  }
};

}

#endif

