/* 
 *  topic_manager.cpp - micros topic manager
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
#include "micros_rtt/topic_manager.h"

namespace micros_rtt 
{

TopicManagerPtr g_topic_manager;

const TopicManagerPtr& TopicManager::instance()
{
  if (!g_topic_manager) 
  {
    if (!g_topic_manager) 
    {
      g_topic_manager.reset(new TopicManager);
    }
  }
  
  return g_topic_manager;
}

TopicManager::TopicManager()
{
}

TopicManager::~TopicManager()
{
}


void TopicManager::addPubConnection(ConnectionBasePtr pub_connection)
{
  //something need to be done for the multi-thread condition
  advertised_topics_.push_back(pub_connection);
}

void TopicManager::addSubConnection(ConnectionBasePtr sub_connection)
{
  //something need to be done for the multi-thread condition
  subscriptions_.push_back(sub_connection);
}

ConnectionBasePtr TopicManager::findSubConnection(const std::string& topic)
{
  // spin through the subscriptions and see if we find a match. if so, use it.
  ConnectionBasePtr sub;

  for (V_ConnectionBase::iterator s = subscriptions_.begin();
         s != subscriptions_.end(); ++s)
  {
    sub = *s;
    if (sub->getTopic() == topic)
    {
      break;
    }
  }

  return sub;
}

ConnectionBasePtr TopicManager::findPubConnection(const std::string& topic)
{
  ConnectionBasePtr pub;
  for (V_ConnectionBase::iterator i = advertised_topics_.begin();
       !pub && i != advertised_topics_.end();
       ++i) 
  {
    if ((*i)->getTopic() == topic) 
    {
      pub = *i;
      break;
    }
  }

  return pub;
}


}

