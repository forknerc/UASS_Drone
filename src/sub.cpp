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

int sockfd;
struct sockaddr_in servaddr,cliaddr;

struct sockaddr_in myaddr;
struct sockaddr_in remaddr;
socklen_t addrlen = sizeof(remaddr);
int recvlen;
int fd;

//////////////////////////

char myID[256];

void chatterCallback(const tum_ardrone::filter_stateConstPtr statePtr)
{

  // format string for output
  char output[2048];
  output[0] = '\0';
  sprintf(output, "3 %s %.4f %.4f %.4f %.4f %.4f %.4f", 
                                                myID,
                                                statePtr->x,
                                                statePtr->z,
                                                statePtr->y,
                                                statePtr->roll,
                                                statePtr->pitch,
                                                statePtr->yaw);
  // send output to ros console
  ROS_INFO(output);

  // send output to socket
  sendto(sockfd,output,strlen(output),0,
             (struct sockaddr *)&servaddr,sizeof(servaddr));


}



int main(int argc, char **argv)
{

  int fakeArgc = 1;
  bool stillWaiting = true;
  myID[0] = '\0';

  ros::init(fakeArgc, argv, "sub");

  ros::NodeHandle n;

  char clientIP[256];
  char buffer[256];

  // get IP address from terminal
  if(argc == 2)
  {
    sprintf(clientIP, "%s", argv[1]);
  }
  else
  {
    printf("Please enter the clients IP address in the cmd arguments\n");
    return 0;
  }

  // init socket
  sockfd = socket(AF_INET,SOCK_DGRAM,0);
  bzero(&servaddr,sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr(clientIP);
  servaddr.sin_port = htons(8053);

  // send join message to 
  char message[256];

  // 0 - message join
  // 0 - unit type: ardrone
  sprintf(message, "0 0"); 

    // send output to socket
  sendto(sockfd,message,strlen(message),0,
             (struct sockaddr *)&servaddr,sizeof(servaddr));


  // init receive socket
  if((fd = socket(AF_INET,SOCK_DGRAM,0)) < 0)
    {
        printf("ERROR CREATING SOCKET\n");
        return 0;
    }

  // bind receive socket to read port 8051
  memset((char*)&myaddr,0,sizeof(myaddr));
  myaddr.sin_family = AF_INET;
  myaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  myaddr.sin_port = htons(8051);

  if(bind(fd, (struct sockaddr *)&myaddr, sizeof(myaddr)) < 0)
  {
      printf("ERROR BINDING SOCKET\n");
  }

  // wait for client to send back unit ID
  while(stillWaiting)
  {
      recvlen = recvfrom(fd, buffer, 256, 0, (struct
                          sockaddr*)&remaddr, &addrlen);
      buffer[recvlen] = '\0';
      printf("received: %s\n", buffer);
      if(buffer[0] == '1')
      {
          stillWaiting = false;
          int i = 2;
          int j = 0;
          while(buffer[i] != '\0' 
                && buffer[i] != ' ' 
                && buffer[i] < 123)
          {
              myID[j] = buffer[i];
              i++;
              j++;
          }
          myID[j] = '\0';
      }
  }

      printf("My ID: %s\n", myID);
//return 0;
  // resolve node name
  std::string predPosChannel = n.resolveName("ardrone/predictedPose");

  ros::Subscriber sub = n.subscribe(predPosChannel, 1000, chatterCallback);

  ros::spin();


  /*bool keepGoing = true;
  ros::Rate r(3);
 while(keepGoing)
    {
        ros::spinOnce();
        // send message over socket
        r.sleep();
    }*/

  return 0;
}
