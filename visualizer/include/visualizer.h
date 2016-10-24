#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <QObject>
#include "types.h"
#include "minho_team_ros/position.h"
#include "minho_team_ros/pose.h"
using minho_team_ros::position;
using minho_team_ros::pose;

class Visualizer : public QObject
{
    Q_OBJECT
public:
   explicit Visualizer(QObject *parent = 0);
public slots:
   // Setup and Output
   void initField(QString file_);
   void drawWorldModel();
   Mat *getWorldModel();
   //setter functions
   void setRobotPosition(pose r_pose);
   void setBallPosition2D(position ball_pos);
private slots:
   void configFilePaths();
   Point world2WorldModel(Point2d pos);
private:
   Mat field,worldModel,relayWModel;
   fieldDimensions fieldAnatomy;
   // File Paths
   QString mainFilePath, configFolderPath;
   QString fieldPath;
   //world state info
   pose robot_pose;
   position ball2DPosition;
};

#endif // VISUALIZER_H
