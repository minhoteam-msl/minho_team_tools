#include "mainwindow.h"
#include <QApplication>
#include <QStyleFactory>
ros::AsyncSpinner *spinner;

int main(int argc, char *argv[])
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
   
   if(mode_real) ROS_INFO("Running Control Calib for Robot %d.",robot_id);
   else ROS_INFO("Running Control Calib for Robot %d in simulation.",robot_id);

   QApplication a(argc, argv);
   a.setStyle(QStyleFactory::create("fusion"));
   MainWindow w(robot_id,mode_real);
   w.show();
   
   spinner = new ros::AsyncSpinner(2);
   spinner->start();

    return a.exec();
}
