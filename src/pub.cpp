#include "ros/ros.h"
#include "ros/package.h"
#include "std_msgs/String.h"
#include "offsetInfo.h"
#include <iostream>
#include <string>
#include <sstream>
#include <algorithm>
#include <iterator>
#include <vector>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
 
using namespace std;

vector<string> Tokenizer(string str);

std_msgs::String StringToMsg(string input);

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pub");
  ros::NodeHandle n;
  std::string predPosChannel = n.resolveName("/tum_ardrone/com");
  ros::Publisher msg_pub = n.advertise<std_msgs::String>(predPosChannel, 1000);
  ros::Publisher offset_pub = n.advertise<std_msgs::String>("UASS_Offset", 1000);
  ros::Rate loop_rate(10);

  int count = 0;
  string in;
  char msgCstr[256];

  UASS::offsetInfo droneOffset;
  droneOffset.X = 0.0f;
  droneOffset.Y = 0.0f;
  droneOffset.Z = 0.0f;
  droneOffset.Yaw = 0.0f;

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

  std_msgs::String msg;
  std_msgs::String msgOffset;

  std::stringstream ss1;
  ss1 << "0 0 0 0";
  msgOffset.data = ss1.str();


  while (ros::ok())
  {
    buff[0] = '\0';    

    recvlen = recvfrom(fd, buff, 256, 0, (struct sockaddr*)&remaddr, &addrlen);

    if(recvlen > 0)
        {
            // add null char
            buff[recvlen] = '\0';
            sprintf(msgCstr, "%s", buff);
            //cout<<newMsg<<endl;
        }

    string stringCpp(msgCstr);

    // parse message
    istringstream iss(stringCpp);
    vector<string> tokens = Tokenizer(stringCpp);
    ROS_INFO("Msg received: %s", msgCstr);
    ROS_INFO("num tokens: %d", tokens.size());

    int cmdType = atoi(tokens[2].c_str());
    int msgType = atoi(tokens[0].c_str());
    bool sendMsg = false;
    msgCstr[0] = '\0';

    if(msgType == 2)
    {
        switch(cmdType)
        {
            case 0:
                if(tokens.size() == 6)
                {
                    sprintf(msgCstr, "c goto %s %s %s 0", tokens[3].c_str(), 
                                                           tokens[5].c_str(), 
                                                           tokens[4].c_str());
                msg = StringToMsg(msgCstr);
                ROS_INFO("Send msg: %s", msg.data.c_str());
                msg_pub.publish(msg);
                }
                break;
            case 1:
                // stop
                msg = StringToMsg("c clearCommands");
                ROS_INFO("Send msg: %s", msg.data.c_str());
                msg_pub.publish(msg);
                break;
            case 2:
                // launch
                msg = StringToMsg("c clearCommands");
                ROS_INFO("Send msg: %s", msg.data.c_str());
                msg_pub.publish(msg);

                msg = StringToMsg("c autoInit 500 800 4000 0.5");
                ROS_INFO("Send msg: %s", msg.data.c_str());
                msg_pub.publish(msg);

                msg = StringToMsg("c setInitialReachDist 0.2");
                ROS_INFO("Send msg: %s", msg.data.c_str());
                msg_pub.publish(msg);

                msg = StringToMsg("c setStayWithinDist 0.3");
                ROS_INFO("Send msg: %s", msg.data.c_str());
                msg_pub.publish(msg);

                msg = StringToMsg("c setStayTime 3");
                ROS_INFO("Send msg: %s", msg.data.c_str());
                msg_pub.publish(msg);

                msg = StringToMsg("c lockScaleFP");
                ROS_INFO("Send msg: %s", msg.data.c_str());
                msg_pub.publish(msg);
                break;
            case 3:
                // land
                msg = StringToMsg("c land");
                ROS_INFO("Send msg: %s", msg.data.c_str());
                msg_pub.publish(msg);
                break;
            case 4:
                // change offset
                droneOffset.X = atof(tokens[3].c_str());
                droneOffset.Y = atof(tokens[5].c_str());
                droneOffset.Z = atof(tokens[4].c_str());
                droneOffset.Yaw = atof(tokens[6].c_str());
                char temp[256];
                sprintf(temp, "%f %f %f %f", droneOffset.X, 
                                             droneOffset.Y,
                                             droneOffset.Z,
                                             droneOffset.Yaw);
                std::stringstream tempStream;
                tempStream << temp;
                msgOffset.data = tempStream.str();
                offset_pub.publish(msgOffset);
                break;
        }
    }
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}

vector<string> Tokenizer(string str)
{
    vector<string> tokens;

    // Skip delimiters at beginning.
    string::size_type lastPos = str.find_first_not_of(" ", 0);
    // Find first "non-delimiter".
    string::size_type pos = str.find_first_of(" ", lastPos);

    while (string::npos != pos || string::npos != lastPos)
    {
        // Found a token, add it to the vector.
        tokens.push_back(str.substr(lastPos, pos - lastPos));
        // Skip delimiters.  Note the "not_of"
        lastPos = str.find_first_not_of(" ", pos);
        // Find next "non-delimiter"
        pos = str.find_first_of(" ", lastPos);
    }
    return tokens;
}

std_msgs::String StringToMsg(string input)
{
    std_msgs::String out;
    std::stringstream ss;
    ss << input;
    out.data = ss.str();
    return out;
}


