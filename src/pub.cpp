#include "ros/ros.h"
#include "std_msgs/String.h"

#include <iostream>
#include <string>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
using namespace std;
/**
 * This tutorial demonstrates simple sending of messages over the ROS system.
 */
int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv so that it can perform
   * any ROS arguments and name remapping that were provided at the command line. For programmatic
   * remappings you can use a different version of init() which takes remappings
   * directly, but for most command-line programs, passing argc and argv is the easiest
   * way to do it.  The third argument to init() is the name of the node.
   *
   * You must call one of the versions of ros::init() before using any other
   * part of the ROS system.
   */
  ros::init(argc, argv, "pub");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to
   * publish on a given topic name. This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing. After this advertise() call is made, the master
   * node will notify anyone who is trying to subscribe to this topic name,
   * and they will in turn negotiate a peer-to-peer connection with this
   * node.  advertise() returns a Publisher object which allows you to
   * publish messages on that topic through a call to publish().  Once
   * all copies of the returned Publisher object are destroyed, the topic
   * will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue
   * used for publishing messages.  If messages are published more quickly
   * than we can send them, the number here specifies how many messages to
   * buffer up before throwing some away.
   */
  std::string predPosChannel = n.resolveName("/tum_ardrone/com");
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>(predPosChannel, 1000);

  ros::Rate loop_rate(10);

  /**
   * A count of how many messages we have sent. This is used to create
   * a unique string for each message.
   */
  int count = 0;
  string in;
  char newMsg[256];

  struct sockaddr_in myaddr;
  struct sockaddr_in remaddr;
  socklen_t addrlen = sizeof(remaddr);
  int recvlen;
  int fd;
  unsigned char buff[256];

  // init socket
  if((fd = socket(AF_INET,SOCK_DGRAM,0)) < 0)
    {
        cerr<<"ERROR CREATING SOCKET"<<endl;
        return 0;
    }

  // bind socket to read port 8052
  memset((char*)&myaddr,0,sizeof(myaddr));
  myaddr.sin_family = AF_INET;
  myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  myaddr.sin_port = htons(8052);

  if(bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0)
    {
        cerr<<"ERROR BINDING SOCKET"<<endl;
    }

  while (ros::ok() && in[0] != 'q')
  {
    /**
     * This is a message object. You stuff it with data, and then publish it.
     */
    std_msgs::String msg;

    recvlen = recvfrom(fd, buff, 256, 0, (struct sockaddr*)&remaddr, &addrlen);

    if(recvlen > 0)
        {
            // add null char
            buff[recvlen] = 0;
            sprintf(newMsg, "%s", buff);
            //cout<<newMsg<<endl;
        }


    std::stringstream ss;
    ss << newMsg << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    /**
     * The publish() function is how you send messages. The parameter
     * is the message object. The type of this object must agree with the type
     * given as a template parameter to the advertise<>() call, as was done
     * in the constructor above.
     */
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}
