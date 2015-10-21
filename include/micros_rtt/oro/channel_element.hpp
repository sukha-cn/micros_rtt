/* 
 *  channel_element.hpp - micros channel element
 *  Copyright (C) 2015 Zaile Jiang
 *  
 *  This program is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU General Public License
 *  as published by the Free Software Foundation; either version 2
 *  of the License, or (at your option) any later version.
 *  
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA.
*/
#ifndef MICROSRTT_CHANNEL_ELEMENT_HPP
#define MICROSRTT_CHANNEL_ELEMENT_HPP

#include <boost/intrusive_ptr.hpp>
#include <boost/call_traits.hpp>
#include "micros_rtt/oro/channel_element_base.hpp"
#include "micros_rtt/common.h"

namespace micros_rtt 
{   
/** A typed version of ChannelElementBase. It defines generic methods that are
 * type-specific (like write and read)
 */
template<typename M>
class ChannelElement : public ChannelElementBase
{
public:
  typedef M value_t;
  typedef boost::intrusive_ptr< ChannelElement<M> > shared_ptr;
  typedef typename boost::call_traits<M>::param_type param_t;
  typedef typename boost::call_traits<M>::reference reference_t;

  shared_ptr getOutput()
  {
    return boost::static_pointer_cast< ChannelElement<M> >(ChannelElementBase::getOutput());
  }

  shared_ptr getInput()
  {
    return boost::static_pointer_cast< ChannelElement<M> >(ChannelElementBase::getInput());
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
    typename ChannelElement<M>::shared_ptr output = boost::static_pointer_cast< ChannelElement<M> >(getOutput());
    if (output)
    {
      return output->data_sample(sample);
    }
    return false;
  }

  virtual value_t data_sample()
  {
    typename ChannelElement<M>::shared_ptr input = boost::static_pointer_cast< ChannelElement<M> >(getInput());
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
    typename ChannelElement<M>::shared_ptr output = boost::static_pointer_cast< ChannelElement<M> >(getOutput());
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
    typename ChannelElement<M>::shared_ptr input = this->getInput();
    if (input)
    {
      return input->read(sample, copy_old_data);
    }
    else
    {
      return NewData;
    }
  }
};

}

#endif

