#include "micros_rtt/publisher.h"

namespace micros_rtt
{

Publisher::Publisher(ros::Publisher ros_publisher, ConnectionBasePtr pub_connection)
{
  publication = pub_connection;
  ros_pub = ros_publisher;
}

}
