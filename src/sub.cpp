#include "ros/ros.h"
#include "std_msgs/String.h"
#include "tum_ardrone/filter_state.h"

#include<cstdio>
#include<cstdlib>

#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */

///////////////SOCKET INIT

int sockfd,n;
struct sockaddr_in servaddr,cliaddr;


//////////////////////////


void chatterCallback(const tum_ardrone::filter_stateConstPtr statePtr)
{

  // format string for output
  char output[2048];
  output[0] = '\0';
  sprintf(output, "x: %.4f y: %.4f z: %.4f roll: %.4f pitch: %.4f yaw: %.4f Battery: %.4f\n", 
                                                statePtr->x,
                                                statePtr->z,
                                                statePtr->y,
                                                statePtr->roll,
                                                statePtr->pitch,
                                                statePtr->yaw,
                                                statePtr->batteryPercent);
  // send output to ros console
  ROS_INFO(output);

  // send output to socket
  sendto(sockfd,output,strlen(output),0,
             (struct sockaddr *)&servaddr,sizeof(servaddr));


}

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
  ros::init(argc, argv, "sub");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle constructed will fully initialize this node, and the last
   * NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  This invokes a call to the ROS
   * master node, which keeps a registry of who is publishing and who
   * is subscribing.  Messages are passed to a callback function, here
   * called chatterCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
   *
   * The second parameter to the subscribe() function is the size of the message
   * queue.  If messages are arriving faster than they are being processed, this
   * is the number of messages that will be buffered up before beginning to throw
   * away the oldest ones.
   */


  sockfd = socket(AF_INET,SOCK_DGRAM,0);

  bzero(&servaddr,sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr("127.0.0.1");
  servaddr.sin_port = htons(8051);


  // resolve node name
  std::string predPosChannel = n.resolveName("ardrone/predictedPose");

  ros::Subscriber sub = n.subscribe(predPosChannel, 1000, chatterCallback);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
  /*bool keepGoing = true;
  ros::Rate r(10);
 while(keepGoing)
    {
        ros::spinOnce();
        // send message over socket
        r.sleep();
    }*/

  return 0;
}
