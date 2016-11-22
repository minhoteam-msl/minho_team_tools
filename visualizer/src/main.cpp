//C++ includes
#include <iostream>
#include <sstream>
#include "ros/ros.h"
//ROS includes
#include "minho_team_ros/robotInfo.h"
#include "minho_team_ros/goalKeeperInfo.h"
#include "minho_team_ros/interAgentInfo.h"
#include "minho_team_ros/requestExtendedDebug.h"
//Application includes
#include <QApplication>
#include "visualizer.h"
#include<arpa/inet.h>
#include<sys/socket.h>
#include "multicast.h"
#include <pthread.h>
#define ROS_MASTER_IP "http://172.16.49."
#define ROS_MASTER_PORT ":11311"
#define BUFFER_SIZE 1024

/// \brief struct to represet a udp packet, containing
/// a serialized ROS message
typedef struct udp_packet{
   uint8_t *packet;
   uint32_t packet_size;
}udp_packet;


using namespace std;
using namespace ros;
using minho_team_ros::robotInfo; //Namespace for robot information msg - SUBSCIBING
using minho_team_ros::goalKeeperInfo;
using minho_team_ros::interAgentInfo;
using minho_team_ros::requestExtendedDebug;

//Node specific objects
/// \brief external visualizer class to draw robot's world model
Visualizer *exvis;
/// \brief boolean variable to control request of extra debug info
bool req_debug;
/// \brieg service client to toogle external debug on target robot
ros::ServiceClient requestExtDebug;
/// \brief id variable of target robot
int robot_id;
/// \brief ROS subscribers for robotInfo and goalKeeperInfo
ros::Subscriber robot_info_sub,gk_info_sub;


// ###### THREAD DATA ######
/// \brief a mutex to avoid multi-thread access to message
pthread_mutex_t exvis_mutex = PTHREAD_MUTEX_INITIALIZER;
/// \brief main udp data receiving thread
pthread_t recv_monitor_thread;
///  \brief buffer to hold received data
uint8_t buffer[BUFFER_SIZE];
// #########################

// ###### SOCKET DATA#######
/// \brief UDP socket descriptor
int socket_fd;
/// \brief received datagram size variable
int recvlen = 0;
/// \brief used ip address in multicast interface : has to be "172.16.49.X"
std::string ip_base; 
/// \brief agent id, based on ip. ip_base.agent_id build up the ip address
/// of the machine. for oficial robots this ranges from 1 to 6, basestation
/// is 7 and the above are dummy id's
uint8_t agent_id;
// #########################

/// \brief robotInfo callback function to receive and process
/// robotInfo messages over ROS.
/// \param msg - pointer to message containing robotInfo 
void robotInfoCallback(const robotInfo::ConstPtr& msg)
{
   static int count = 0;
   pthread_mutex_lock (&exvis_mutex); //Lock mutex
	exvis->setRobotInfo(*msg);
	exvis->drawWorldModel();
	pthread_mutex_unlock (&exvis_mutex); //Unlock mutex
	
	if(count<10){
      count++;
   }else {
      count = 0;
      requestExtendedDebug srv;
      srv.request.requested = req_debug;
      requestExtDebug.call(srv); 
   } 
}

/// \brief goalKeeperInfo callback function to receive and process
/// goalKeeperInfo messages over ROS.
/// \param msg - pointer to message containing goalKeeperInfo 
void goalKeeperInfoCallback(const goalKeeperInfo::ConstPtr& msg)
{
   static int count = 0;
   robotInfo robot_info_msg = msg->robot_info;
   pthread_mutex_lock (&exvis_mutex); //Lock mutex
	exvis->setRobotInfo(robot_info_msg);
	exvis->drawWorldModel();
	pthread_mutex_unlock (&exvis_mutex); //Unlock mutex
	   
   if(count<10){
      count++;
   }else {
      count = 0;
      requestExtendedDebug srv;
      srv.request.requested = req_debug;
      requestExtDebug.call(srv); 
   } 
}

/// \brief deserializes a string of bytes into a message
/// \typename Message - type of ROS mesasge to deserialize
/// \param packet - uint8_t vector containing message to deserialize
/// \param msg - pointer to interAgentInfo destination object
/// WARN : ONLY FOR INTERAGENT MESSAGES (SENT WITH UDP)
template<typename Message>
void deserializeROSMessage(udp_packet *packet, Message *msg)
{  
   ros::serialization::IStream istream(packet->packet, packet->packet_size);
   ros::serialization::deserialize(istream, *msg);
}

/// \brief main udp receiving thread. This thread deals with datagram reception
/// and sends the received data to be dealt by a thread using threadpool
/// \param socket - pointer to socket descriptor to use 
void* udpReceivingThread(void *socket)
{
   ROS_INFO("Multicast receiving thread started ...");
   while(ros::ok()){
      bzero(buffer, BUFFER_SIZE);   
      if((recvlen = recv(*(int*)socket, buffer, BUFFER_SIZE,0)) > 0 ){
         udp_packet relay_packet;
         relay_packet.packet = new uint8_t[recvlen];
         relay_packet.packet_size = recvlen;
         memcpy(relay_packet.packet,buffer,recvlen);
         interAgentInfo msg;
         deserializeROSMessage<interAgentInfo>(&relay_packet,&msg);
         if(msg.agent_id==robot_id){
            pthread_mutex_lock (&exvis_mutex); //Lock mutex
	         exvis->setRobotInfo(msg.agent_info.robot_info);
	         exvis->drawWorldModel();
            pthread_mutex_unlock (&exvis_mutex); //Unlock mutex 
         }
      }
   }
   
   return NULL;
}

int main(int argc, char **argv)
{
   if(argc!=3) { 
      ROS_ERROR("Must enter robot id and mode as parameter.\n \
      Please use -r for real robot (using ROS),-s for simulation or -m using multicast, followed by the robot's ID.");
      exit(1); 
   }

   robot_id = QString::fromLocal8Bit(argv[2]).toInt();
   if(robot_id<0 || robot_id>6){
      ROS_ERROR("Must enter robot id correctly. Robot id's range from 1 to 6 and 0 to localhost.");
      exit(2);     
   }

   int mode_type = 0;
   QString mode = QString::fromLocal8Bit(argv[1]);
   if(mode == "-r") mode_type = 0;
   else if(mode == "-s") mode_type = 1;
   else if(mode == "-m") mode_type = 2;
   else { 
      ROS_ERROR("Must enter mode correctly. Please use -r for real robot (using ROS),-s for simulation or -m using multicast.");
      exit(3); 
   }
   
   QApplication a(argc, argv);
   exvis = new Visualizer(robot_id);
   req_debug = false;
   std::stringstream robot_info_topic;
   std::stringstream goalkeeper_info_topic;
   std::stringstream request_debug_topic;
   std::stringstream node_name;
   node_name << "visualizer_" << std::time(0);
   
   if(mode_type==1) { // ROS to Simulated robot
      ROS_INFO("Running visualizer for Robot %d in simulation.",robot_id);
      robot_info_topic << "minho_gazebo_robot" << robot_id;
      goalkeeper_info_topic << "minho_gazebo_robot" << robot_id;
      request_debug_topic << "minho_gazebo_robot" << robot_id;
   } else if(mode_type==0) {  // ROS to Real robot
      ROS_INFO("Running visualizer for Robot %d.",robot_id);
      if(robot_id>0){
         // Setup custom master
         QString robot_ip = QString(ROS_MASTER_IP)+QString::number(robot_id)
                               +QString(ROS_MASTER_PORT);
         ROS_WARN("ROS_MASTER_URI: '%s'",robot_ip.toStdString().c_str());
         setenv("ROS_MASTER_URI",robot_ip.toStdString().c_str(),1);
      } else ROS_WARN("ROS_MASTER_URI is localhost");
   } else {  // coms_node to Real robot
      ROS_INFO("Running visualizer for Robot %d via Multicast.",robot_id);
      socket_fd = openSocket("wlan0",&ip_base,&agent_id,1);
      if(socket_fd<0) exit(0);
      ROS_INFO("UDP Multicast System started.");  
      // start receiving thread
      pthread_create(&recv_monitor_thread, NULL, udpReceivingThread, &socket_fd);
   }
   
   robot_info_topic << "/robotInfo";
   goalkeeper_info_topic << "/goalKeeperInfo";
	request_debug_topic << "/requestExtendedDebug";
	//Initialize ROS
	ros::init(argc, argv, node_name.str().c_str(),ros::init_options::NoSigintHandler);
	//Request node handler
	ros::NodeHandle visualizer;
	
	if(mode_type<=1){
	   ROS_WARN("Subscribing to robotInfo & goalkeeperInfo.");
	   //Initialize hardwareInfo subscriber
	   robot_info_sub = visualizer.subscribe(robot_info_topic.str(), 100, robotInfoCallback);
	   gk_info_sub = visualizer.subscribe(goalkeeper_info_topic.str(), 100, robotInfoCallback);
	   requestExtDebug = visualizer.serviceClient<minho_team_ros::requestExtendedDebug>(request_debug_topic.str().c_str());
	}
		
	if(mode_type<=1)ROS_WARN("MinhoTeam visualizer started running on ROS.");
	else ROS_WARN("MinhoTeam visualizer started over Multicast.");

   namedWindow("World Model");
	ros::AsyncSpinner spinner(2);
	spinner.start();
	int ret = 0;
	while(ros::ok()){
		Mat *wmodel = exvis->getWorldModel();
		imshow("World Model",*wmodel);
		ret = waitKey(20);
		if(ret==27){ //ESC
		   ROS_ERROR("ESC pressed. Closing ...");
		   ros::shutdown();
		   destroyWindow("World Model");
		   a.quit();
		   return 0;
		} else if(ret=='i' || ret=='I'){
		   req_debug = !req_debug;
		   if(req_debug) ROS_INFO("Extended debug set to ON");
		   else ROS_INFO("Extended debug set to OFF");
		} else if(ret=='o' || ret=='O'){
		   ROS_INFO("Changing to Official Field.");
		   exvis->initField(exvis->getFieldFileName("Official"));
		} else if(ret=='l' || ret=='L'){
		   ROS_INFO("Changing to Lar Field.");
		   exvis->initField(exvis->getFieldFileName("Lar"));
		}
	}
	
	pthread_join(recv_monitor_thread, NULL);
	closeSocket(socket_fd);
	return a.exec();
	return 0;
}
