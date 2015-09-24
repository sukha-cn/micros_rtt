/* 
 *  channel_data_element.hpp - micros channel data element
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
#ifndef MICROSRTT_CHANNEL_DATA_ELEMENT_HPP
#define MICROSRTT_CHANNEL_DATA_ELEMENT_HPP

#include "micros_rtt/oro/channel_element.hpp"
#include "micros_rtt/oro/data_lockfree.hpp"

namespace micros_rtt 
{    
  
/** A connection element that stores a single data sample */
template<typename T>
class ChannelDataElement : public ChannelElement<T>
{
  bool written, mread;
  typename DataObjectLockFree<T>::shared_ptr data;

public:
  typedef typename ChannelElement<T>::param_t param_t;
  typedef typename ChannelElement<T>::reference_t reference_t;

  ChannelDataElement(typename DataObjectLockFree<T>::shared_ptr sample)
        : written(false), mread(false), data(sample) {}

  /** Update the data sample stored in this element.
   * It always returns true. */
  virtual bool write(param_t sample)
  {
    data->Set(sample);
    written = true;
    mread = false;
    return this->signal();
  }

  /** Reads the last sample given to write()
   *
   * @return false if no sample has ever been written, true otherwise
   */
  virtual FlowStatus read(reference_t sample, bool copy_old_data)
  {
    if (written)
    {
      if ( !mread ) {
        data->Get(sample);
        mread = true;
        return NewData;
      }

      if(copy_old_data)
        data->Get(sample);

      return OldData;
    }
    return NoData;
  }

  /** Resets the stored sample. After clear() has been called, read()
   * returns false
   */
  virtual void clear()
  {
    written = false;
    mread = false;
    ChannelElement<T>::clear();
  }

  virtual bool data_sample(param_t sample)
  {
    data->data_sample(sample);
    return ChannelElement<T>::data_sample(sample);
  }

  virtual T data_sample()
  {
    return data->Get();
  }

};

}

#endif

