//C++ includes
#include <iostream>
#include <sstream>
#include "ros/ros.h"
//ROS includes
#include "minho_team_ros/robotInfo.h"
#include "minho_team_ros/goalKeeperInfo.h"
#include "minho_team_ros/requestExtendedDebug.h"
//Application includes
#include <QApplication>
#include "visualizer.h"
#define ROS_MASTER_IP "http://172.16.49."
#define ROS_MASTER_PORT ":11311"


using namespace std;
using namespace ros;
using minho_team_ros::robotInfo; //Namespace for robot information msg - SUBSCIBING
using minho_team_ros::goalKeeperInfo;
using minho_team_ros::requestExtendedDebug;

//Node specific objects
Visualizer *exvis;
bool req_debug;
ros::ServiceClient requestExtDebug;

void robotInfoCallback(const robotInfo::ConstPtr& msg)
{
   static int count = 0;
	exvis->setRobotInfo(*msg);
	exvis->drawWorldModel();
	if(count<10){
      count++;
   }else {
      count = 0;
      requestExtendedDebug srv;
      srv.request.requested = req_debug;
      requestExtDebug.call(srv); 
   } 
}

void goalKeeperInfoCallback(const goalKeeperInfo::ConstPtr& msg)
{
   static int count = 0;
   robotInfo robot_info_msg = msg->robot_info;
	exvis->setRobotInfo(robot_info_msg);
	exvis->drawWorldModel();
   if(count<10){
      count++;
   }else {
      count = 0;
      requestExtendedDebug srv;
      srv.request.requested = req_debug;
      requestExtDebug.call(srv); 
   } 
}

int main(int argc, char **argv)
{
   if(argc!=3) { 
      ROS_ERROR("Must enter robot id and mode as parameter.\n \
      Please use -r for real robot or -s for simulation, followed by the robot's ID.");
      exit(1); 
   }

   int robot_id = QString::fromLocal8Bit(argv[2]).toInt();
   if(robot_id<0 || robot_id>6){
      ROS_ERROR("Must enter robot id correctly. Robot id's range from 1 to 6 and 0 to localhost.");
      exit(2);     
   }

   bool mode_real = false;
   QString mode = QString::fromLocal8Bit(argv[1]);
   if(mode == "-r") mode_real = true;
   else if(mode == "-s") mode_real = false;
   else { 
      ROS_ERROR("Must enter mode correctly. Please use -r for real robot or -s for simulation.");
      exit(3); 
   }
   
   QApplication a(argc, argv);
   req_debug = false;
   std::stringstream robot_info_topic;
   std::stringstream goalkeeper_info_topic;
   std::stringstream request_debug_topic;
   
   if(!mode_real) {
      ROS_INFO("Running Vision Calib for Robot %d in simulation.",robot_id);
      robot_info_topic << "minho_gazebo_robot" << robot_id;
      goalkeeper_info_topic << "minho_gazebo_robot" << robot_id;
      request_debug_topic << "minho_gazebo_robot" << robot_id;
   } else {
      ROS_INFO("Running Vision Calib for Robot %d.",robot_id);
      if(robot_id>0){
         // Setup custom master
         QString robot_ip = QString(ROS_MASTER_IP)+QString::number(robot_id)
                               +QString(ROS_MASTER_PORT);
         ROS_WARN("ROS_MASTER_URI: '%s'",robot_ip.toStdString().c_str());
         setenv("ROS_MASTER_URI",robot_ip.toStdString().c_str(),1);
      } else ROS_WARN("ROS_MASTER_URI is localhost");
   }
   
   robot_info_topic << "/robotInfo";
   goalkeeper_info_topic << "/goalKeeperInfo";
	request_debug_topic << "/requestExtendedDebug";
	//Initialize ROS
	ros::init(argc, argv, "visualizer",ros::init_options::NoSigintHandler);
	//Request node handler
	ros::NodeHandle visualizer;
	ROS_WARN("Subscribing to robotInfo & goalkeeperInfo.");
	//Initialize hardwareInfo subscriber
	
	ros::Subscriber robot_info_sub = visualizer.subscribe(robot_info_topic.str(), 100, robotInfoCallback);
	ros::Subscriber gk_info_sub = visualizer.subscribe(goalkeeper_info_topic.str(), 100, robotInfoCallback);
	requestExtDebug = visualizer.serviceClient<minho_team_ros::requestExtendedDebug>(request_debug_topic.str().c_str());
	
	exvis = new Visualizer(robot_id);	
	ROS_WARN("MinhoTeam visualizer started running on ROS.");

   namedWindow("World Model");
	ros::AsyncSpinner spinner(2);
	spinner.start();
	int ret = 0;
	while(ros::ok()){
		Mat *wmodel = exvis->getWorldModel();
		imshow("World Model",*wmodel);
		ret = waitKey(5);
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
		}
	}
	return a.exec();
	return 0;
}
