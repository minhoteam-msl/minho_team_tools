//C++ includes
#include <iostream>
#include <sstream>
#include "ros/ros.h"
//ROS includes
#include "minho_team_ros/robotInfo.h"
//Application includes
#include <QApplication>
#include "visualizer.h"
#define ROS_MASTER_IP "http://172.16.49."
#define ROS_MASTER_PORT ":11311"


using namespace std;
using namespace ros;
using minho_team_ros::robotInfo; //Namespace for robot information msg - SUBSCIBING

//Node specific objects
Visualizer *exvis;
void robotInfoCallback(const robotInfo::ConstPtr& msg)
{
	exvis->setRobotInfo(*msg);
	exvis->drawWorldModel();
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

   if(mode_real) ROS_INFO("Running Vision Calib for Robot %d.",robot_id);
   else ROS_INFO("Running Vision Calib for Robot %d in simulation.",robot_id);
   if(robot_id>0){
      // Setup custom master
      QString robot_ip = QString(ROS_MASTER_IP)+QString::number(robot_id)
                            +QString(ROS_MASTER_PORT);
      ROS_WARN("ROS_MASTER_URI: '%s'",robot_ip.toStdString().c_str());
      setenv("ROS_MASTER_URI",robot_ip.toStdString().c_str(),1);
   } else ROS_WARN("ROS_MASTER_URI is localhost");
   
	QApplication a(argc, argv);
	//Initialize ROS
	ros::init(argc, argv, "visualizer",ros::init_options::NoSigintHandler);
	//Request node handler
	ros::NodeHandle visualizer;
	ROS_WARN("Subscribing to robotInfo & goalkeeperInfo.");
	//Initialize hardwareInfo subscriber
	/*ros::Subscriber goalkeeper_info_sub = externalvis_node.subscribe("goalkeeperInfo", 1000, goalkeeperInfoCallback);*/
	std::stringstream robot_info_topic;
	if(mode_real){
	   robot_info_topic << "robotInfo";
	} else {
	   robot_info_topic << "minho_robot_" << robot_id << "/robotInfo";
	}
	ros::Subscriber robot_info_sub = visualizer.subscribe(robot_info_topic.str(), 1000, robotInfoCallback);
	exvis = new Visualizer(robot_id);	
	ROS_WARN("MinhoTeam visualizer started running on ROS.");


	ros::AsyncSpinner spinner(2);
	spinner.start();
	
	while(ros::ok()){
		Mat *wmodel = exvis->getWorldModel();
		imshow("World Model",*wmodel);
		waitKey(5);
	}
	return a.exec();
	return 0;
}
