#ifndef MICROSRTT_TOPIC_MANAGER_H
#define MICROSRTT_TOPIC_MANAGER_H

#include "ros/ros.h"
#include "common.h"
#include "micros_rtt/publication.h"
#include "micros_rtt/subscription.h"
#include "micros_rtt/connection_base.hpp"
#include "micros_rtt/oro/connection_factory.hpp"

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

  template<class M> 
  ConnectionBasePtr advertise(const std::string topic, bool is_interprocess)
  {
    ConnectionBasePtr pub;
    pub = lookupPublication(topic);
    if (pub) 
    {
      if (pub->isInterprocess() == is_interprocess)
      {
        return pub;
      }
      else
      {
        ROS_INFO("topic has been published as the other method.");
        return pub;
      }
    }
  
    if (is_interprocess)
    {
      pub.reset(new InterPublication<M>(topic));
      advertised_topics_.push_back(pub);
      ConnFactory::createStream<M>(pub, topic, true);
    }
    else 
    {
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
    } 
    return pub;
  }
  //template<class M> bool unadvertise(const std::string topic);

  template<class M> 
  ConnectionBasePtr subscribe(const std::string& topic, void(*fp)(M), bool is_interprocess = false)
  {
    boost::function<void(M)> callback = fp;
    ConnectionBasePtr s = lookupSubscription(topic);
    if (s)
    {
      ROS_INFO("topic manager found topic.");
      return s;
    }
    
    if (is_interprocess)
    {
      ROS_INFO("topic manager creating inter subscription.");
      s.reset(new InterSubscription<M>(topic, callback));
      ConnFactory::createStream<M>(s, topic, false);
    }
    else 
    {
      s.reset(new Subscription<M>(topic, callback));
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
    }


    subscriptions_.push_back(s);
    
    return s;
  }
  //bool unsubscribe(const std::string& topic, const std::string& data_type);
  
  ConnectionBasePtr lookupPublication(const std::string& topic);
  ConnectionBasePtr lookupSubscription(const std::string& topic);
  
private:
  V_ConnectionBase advertised_topics_;
  V_ConnectionBase subscriptions_;

};

}

#endif
