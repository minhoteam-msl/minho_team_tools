#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <QObject>
#include "types.h"
#include "minho_team_ros/robotInfo.h"
using minho_team_ros::robotInfo;

class Visualizer : public QObject
{
    Q_OBJECT
public:
   explicit Visualizer(int id, QObject *parent = 0);
public slots:
   // Setup and Output
   void initField(QString file_);
   void drawWorldModel();
   Mat *getWorldModel();
   //setter functions
   void setRobotInfo(robotInfo msg);
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
   robotInfo robot_info;
   int robot_id;
};

#endif // VISUALIZER_H
