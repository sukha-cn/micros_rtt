#include "hpcl_rtt/publisher.h"

namespace hpcl_rtt
{
Publisher::Publisher()
{
 
}
 
Publisher::Publisher(ros::Publisher ros_publisher)
{
  ros_pub = ros_publisher;
}

Publisher::Publisher(ConnectionBasePtr pub_connection, ros::Publisher ros_publisher)
{
  publication = pub_connection;
  ros_pub = ros_publisher;
}

Publisher::~Publisher()
{
 
}


}
