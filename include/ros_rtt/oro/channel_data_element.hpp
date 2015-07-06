#ifndef ROSRTT_CHANNEL_DATA_ELEMENT_HPP
#define ROSRTT_CHANNEL_DATA_ELEMENT_HPP

#include "channel_element.hpp"
#include "data_lockfree.hpp"

namespace rosrtt 
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

