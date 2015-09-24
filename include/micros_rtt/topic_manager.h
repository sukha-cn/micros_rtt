/* 
 *  topic_manager.h - micros topic manager
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
#ifndef MICROSRTT_TOPIC_MANAGER_H
#define MICROSRTT_TOPIC_MANAGER_H

#include "ros/ros.h"
#include "micros_rtt/common.h"
#include "micros_rtt/connection_base.h"

namespace micros_rtt
{
class TopicManager;
typedef boost::shared_ptr<TopicManager> TopicManagerPtr;

class TopicManager
{
public:
  TopicManager();
  ~TopicManager();
  static const TopicManagerPtr& instance();
  
  void addPubConnection(ConnectionBasePtr pub_connection);
  void addSubConnection(ConnectionBasePtr sub_connection);
  ConnectionBasePtr findSubConnection(const std::string& topic);
  ConnectionBasePtr findPubConnection(const std::string& topic);
    
private:
  V_ConnectionBase advertised_topics_;
  V_ConnectionBase subscriptions_;

};

}

#endif
