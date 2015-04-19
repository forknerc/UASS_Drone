#include "ros/ros.h"
#include "ros/package.h"
#include "ros/service_callback_helper.h"
#include "std_msgs/String.h"
#include "std_srvs/Empty.h"
#include "std_msgs/Empty.h"
#include <geometry_msgs/Twist.h>
#include "offsetInfo.h"
#include "tum_ardrone/filter_state.h"
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
#include <cstdlib>
 
using namespace std;

vector<string> Tokenizer(string str);

std_msgs::String StringToMsg(string input);

void updatePosition(const tum_ardrone::filter_stateConstPtr statePtr);

float posZ;

int main(int argc, char **argv)
{
  posZ = 0;

  ros::init(argc, argv, "pub");
  ros::NodeHandle n;
  std::string comChannel = n.resolveName("/tum_ardrone/com");
  ros::Publisher msg_pub = n.advertise<std_msgs::String>(comChannel, 1000);
  ros::Publisher offset_pub = n.advertise<std_msgs::String>("UASS_Offset", 1000);
  ros::Publisher UASS_SetKB = n.advertise<std_msgs::Empty>("UASS_SetKB", 1000);
  ros::Publisher UASS_SetAI = n.advertise<std_msgs::Empty>("UASS_SetAI", 1000);


  ros::Publisher autonomy_move = n.advertise<geometry_msgs::Twist>("cmd_vel",1000);
  ros::Publisher autonomy_takeoff = n.advertise<std_msgs::Empty>("ardrone/takeoff",1000);
  ros::Publisher autonomy_land = n.advertise<std_msgs::Empty>("ardrone/land",1000);

  std::string predPosChannel = n.resolveName("ardrone/predictedPose");
  ros::Subscriber sub = n.subscribe(predPosChannel, 1000, updatePosition);

  ros::Rate loop_rate(1000);

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
    bool isEmergency = false;

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
        msg.data = "";

        float anewX = 0.0;
        float anewY = 0.0;
        float anewZ = 0.0;
        geometry_msgs::Twist newTwist;
        int direction;

        switch(cmdType)
        {
            case 0:
                // goto
                if(tokens.size() == 6 && !isEmergency)
                {
                    UASS_SetAI.publish(std_msgs::Empty());

                    double newX = atof(tokens[3].c_str());
                    double newY = atof(tokens[5].c_str());
                    double newZ = posZ;

                    // find direction the drone needs to move

                    sprintf(msgCstr, "c goto %.3f %.3f %.3f 0", newX, 
                                                                newY, 
                                                                newZ);
                    msg = StringToMsg(msgCstr);
                    ROS_INFO("Send msg: %s", msg.data.c_str());
                    msg_pub.publish(msg);
                }
                break;
            case 1:
                UASS_SetKB.publish(std_msgs::Empty());
                // stop
                newTwist.linear.x = 0.0;
                newTwist.linear.y = 0.0;
                newTwist.linear.z = 0.0;
                newTwist.angular.x = 0.0;
                newTwist.angular.y = 0.0;
                newTwist.angular.z = 0.0;
                autonomy_move.publish(newTwist);

                msg = StringToMsg("c clearCommands");
                ROS_INFO("Send msg: %s", msg.data.c_str());
                msg_pub.publish(msg);
                break;
            case 2:
                // launch
                if(!isEmergency)
                {
                    UASS_SetAI.publish(std_msgs::Empty());
                    //autonomy_takeoff.publish(std_msgs::Empty());

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

                    msg = StringToMsg("c setStayTime 2");
                    ROS_INFO("Send msg: %s", msg.data.c_str());
                    msg_pub.publish(msg);

                    msg = StringToMsg("c lockScaleFP");
                    ROS_INFO("Send msg: %s", msg.data.c_str());
                    msg_pub.publish(msg);
                }
                break;
            case 3:
                // land
                /*msg = StringToMsg("c clearCommands");
                ROS_INFO("Send msg: %s", msg.data.c_str());
                msg_pub.publish(msg);*/
                //if(!isEmergency)
                //{
                    UASS_SetAI.publish(std_msgs::Empty());
                    //autonomy_land.publish(std_msgs::Empty());
            
                    msg = StringToMsg("c land");
                    ROS_INFO("Send msg: %s", msg.data.c_str());
                    msg_pub.publish(msg);
                //}
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
                msgOffset = StringToMsg(temp);
                offset_pub.publish(msgOffset);
                break;
            case 5:
                UASS_SetKB.publish(std_msgs::Empty());

                direction = atoi(tokens[3].c_str());

                switch(direction)
                {
                    case 1:
                        anewX = 0.353;
                        anewY = 0.353;
                        anewZ = 0.0;
                        break;
                    case 2:
                        anewX = 0.5;
                        anewY = 0.0;
                        anewZ = 0.0;
                        break;
                    case 3:
                        anewX = 0.353;
                        anewY = -0.353;
                        anewZ = 0.0;
                        break;
                    case 4:
                        anewX = 0.0;
                        anewY = -0.5;
                        anewZ = 0.0;
                        break;
                    case 5:
                        anewX = -0.353;
                        anewY = -0.353;
                        anewZ = 0.0;
                        break;
                    case 6:
                        anewX = -0.5;
                        anewY = 0.0;
                        anewZ = 0.0;
                        break;
                    case 7:
                        anewX = -0.353;
                        anewY = 0.353;
                        anewZ = 0.0;
                        break;
                    case 8:
                        anewX = 0.0;
                        anewY = 0.5;
                        anewZ = 0.0;
                        break;
                    case 9:
                        anewX = 0.0;
                        anewY = 0.0;
                        anewZ = 0.5;
                        break;
                    case 10:
                        anewX = 0.0;
                        anewY = 0.0;
                        anewZ = -0.5;
                        break;
                    default:
                        break;
                }
                newTwist.linear.x = anewX;
                newTwist.linear.y = anewY;
                newTwist.linear.z = anewZ;
                newTwist.angular.x = 0.0;
                newTwist.angular.y = 0.0;
                newTwist.angular.z = 0.0;
                
                autonomy_move.publish(newTwist);
                break;

            case 6: // emergency stop
                isEmergency = true;
                //autonomy_land.publish(std_msgs::Empty());

                ROS_INFO("Recieved Emegrency, shutting down");
                msg = StringToMsg("c clearCommands");
                ROS_INFO("Send msg: %s", msg.data.c_str());
                msg_pub.publish(msg);

                msg = StringToMsg("c land");
                ROS_INFO("Send msg: %s", msg.data.c_str());
                msg_pub.publish(msg);
                
                break;
            case 7: // emergency end
                ROS_INFO("Recieved Emegrency end, starting up");
                isEmergency = false;
                break;
            case 8: // DO A BARREL ROLL!
                system("rosservice call /ardrone/setflightanimation 18 0");
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

void updatePosition(const tum_ardrone::filter_stateConstPtr statePtr)
{
    posZ = statePtr->z;
}


