#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Empty.h"
#include "std_srvs/Empty.h"

#include <iostream>
#include <string>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

using namespace std;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "UASSflatTrim");

  ros::NodeHandle n;

  std::string flatTrimChannel = n.resolveName("ardrone/flattrim");

  ROS_INFO("%s", flatTrimChannel.c_str());

  ros::ServiceClient flattrim_srv = n.serviceClient<std_srvs::Empty>(n.resolveName("ardrone/flattrim"),1);

  std_srvs::Empty flattrim_srv_srvs;

  ros::Rate loop_rate(10);

  char input = ' ';
  std_msgs::String msg;
  msg.data = " ";

  while (ros::ok() && input != 'q')
  {
    cin>>input;  

    ROS_INFO("sending flat trim");

    flattrim_srv.call(flattrim_srv_srvs);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
