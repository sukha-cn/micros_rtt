#ifndef ROSRTT_TOPIC_MANAGER_H
#define ROSRTT_TOPIC_MANAGER_H

#include "ros/ros.h"
#include "common.h"
#include "hpcl_rtt/publication.h"
#include "hpcl_rtt/subscription.h"
#include "hpcl_rtt/connection_base.h"
#include "hpcl_rtt/oro/connection_factory.hpp"

namespace hpcl_rtt
{
class TopicManager;
typedef boost::shared_ptr<TopicManager> TopicManagerPtr;

class TopicManager
{
public:
  TopicManager();
  ~TopicManager();
  static const TopicManagerPtr& instance();

  template<class M> 
  ConnectionBasePtr advertise(const std::string topic)
  {
    ConnectionBasePtr pub;
    pub = lookupPublication(topic);
    if (pub) 
    {
      return pub;
    }

    pub.reset(new Publication<M>(topic));
    advertised_topics_.push_back(pub);

    // check whether we've already subscribed to this topic.
    bool found = false;
    ConnectionBasePtr sub;
    {
      V_ConnectionBase::iterator s;
      for (s = subscriptions_.begin(); s != subscriptions_.end() && !found; ++s) 
      {
        if ((*s)->getTopic() == topic) 
        {
          found = true;
          sub = *s;
          break;
        }
      }
    }

    if (found) 
    {
      ConnFactory::createConnection<M>(pub, sub);
    }

    return pub;
  }
  //template<class M> bool unadvertise(const std::string topic);

  template<class M> 
  ConnectionBasePtr subscribe(const std::string& topic, void(*fp)(M))
  {
    boost::function<void(M)> callback = fp;
    boost::shared_ptr< Subscription<M> > s(new Subscription<M>(topic, callback));

    //add local connection
    bool self_subscribed = false;
    ConnectionBasePtr pub;
    V_ConnectionBase::const_iterator it = advertised_topics_.begin();
    V_ConnectionBase::const_iterator end = advertised_topics_.end();
    for (; it != end; ++it)
    {
      pub = *it;

      if (pub->getTopic() == s->getTopic())
      {
        ROS_INFO("find topic.");
        self_subscribed = true;
        break;
      }
    }
    if (self_subscribed)
    {
      if (ConnFactory::createConnection<M>(pub, s))
        ROS_INFO("connected.");
    }

    subscriptions_.push_back(s);
    
    return s;
  }
  //bool unsubscribe(const std::string& topic, const std::string& data_type);
  
  ConnectionBasePtr lookupPublication(const std::string& topic);
  
private:
  V_ConnectionBase advertised_topics_;
  V_ConnectionBase subscriptions_;

};

}

#endif
