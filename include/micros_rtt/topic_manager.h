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
