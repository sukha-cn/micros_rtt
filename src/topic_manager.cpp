#include "micros_rtt/topic_manager.h"


namespace micros_rtt 
{

TopicManagerPtr g_topic_manager;

const TopicManagerPtr& TopicManager::instance()
{
  if (!g_topic_manager) 
  {
//    boost::mutex::scoped_lock lock(g_topic_manager_mutex);
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

ConnectionBasePtr TopicManager::lookupPublication(const std::string& topic)
{
  ConnectionBasePtr t;
  for (V_ConnectionBase::iterator i = advertised_topics_.begin();
       !t && i != advertised_topics_.end();
       ++i) 
  {
    if ((*i)->getTopic() == topic) 
    {
      t = *i;
      break;
    }
  }

  return t;
}


}

