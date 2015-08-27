
#include "micros_rtt/oro/channel_data_element.hpp"
#include "micros_rtt/oro/oro_arch.h"
//#include "../os/MutexLock.hpp"

namespace micros_rtt
{

ChannelElementBase::ChannelElementBase()
    : input(0)
{
  ORO_ATOMIC_SETUP(&refcount,0);
}

ChannelElementBase::~ChannelElementBase()
{
  ORO_ATOMIC_CLEANUP(&refcount);
}

ChannelElementBase::shared_ptr ChannelElementBase::getInput()
{ 
//  RTT::os::MutexLock lock(inout_lock);
  return ChannelElementBase::shared_ptr(input);
}

ChannelElementBase::shared_ptr ChannelElementBase::getOutput()
{ 
  //RTT::os::MutexLock lock(inout_lock);
  return ChannelElementBase::shared_ptr(output);
}

void ChannelElementBase::setOutput(shared_ptr output)
{
  this->output = output;
  if (output)
    output->input = this;
}

void ChannelElementBase::disconnect(bool forward)
{
  if (forward)
  {
    shared_ptr output = getOutput();
    if (output)
      output->disconnect(true);
  }
  else
  {
    shared_ptr input = getInput();
    if (input)
      input->disconnect(false);
  }

//  { RTT::os::MutexLock lock(inout_lock);
    this->input = 0;
    this->output = 0;
//  }
}

ChannelElementBase::shared_ptr ChannelElementBase::getInputEndPoint()
{
  shared_ptr input = getInput();
  return input ? input->getInputEndPoint() : this;
}
ChannelElementBase::shared_ptr ChannelElementBase::getOutputEndPoint()
{
  shared_ptr output = getOutput();
  return output ? output->getOutputEndPoint() : this;
}

bool ChannelElementBase::inputReady()
{
  // we go against the data stream
  shared_ptr input = getInput();
  if (input)
    return input->inputReady();
  return false;
}

void ChannelElementBase::clear()
{
  shared_ptr input = getInput();
  if (input)
    input->clear();
}

bool ChannelElementBase::signal()
{
  shared_ptr output = getOutput();
  if (output)
    return output->signal();
  return true;
}

void ChannelElementBase::ref()
{
  oro_atomic_inc(&refcount);
}

void ChannelElementBase::deref()
{
  if ( oro_atomic_dec_and_test(&refcount) ) delete this;
}

void intrusive_ptr_add_ref( ChannelElementBase* p )
{ p->ref(); }

void intrusive_ptr_release( ChannelElementBase* p )
{ p->deref(); }

}
