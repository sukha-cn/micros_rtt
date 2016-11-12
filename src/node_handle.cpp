/* 
 *  node_handle.hpp - micros node handle
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

#include "micros_rtt/node_handle.hpp"

namespace micros_rtt {

void NodeHandle::setCallbackQueue(ros::CallbackQueueInterface* queue)
{
  ros_nh_.setCallbackQueue(queue);
}

ros::CallbackQueueInterface* NodeHandle::getCallbackQueue() const
{
  return ros_nh_.getCallbackQueue();
}

const std::string& NodeHandle::getNamespace() const 
{
  return ros_nh_.getNamespace();
}

const std::string& NodeHandle::getUnresolvedNamespace()
{
  return ros_nh_.getUnresolvedNamespace();
}

std::string NodeHandle::resolveName(const std::string& name, bool remap) const
{
  return ros_nh_.getUnresolvedNamespace();
}

}