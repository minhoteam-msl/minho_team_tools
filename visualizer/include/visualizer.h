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
   /// \brief constructor of the class. Calls initialization functions.
   /// Calls file path initialization, calling also the function that
   /// draws the initial field
   /// \param id - Id of the robot that will be visualized
   explicit Visualizer(int id, QObject *parent = 0);
public slots:
   /// \brief function that returns a pointer to the current image 
   /// representing the world model described by the robot
   /// \return pointer to the matrix realWModel
   Mat *getWorldModel();
   
   /// \brief initializes field image given the field dimensions and
   /// data in a file.
   /// \param file_ - path of the file containing field dimensions
   void initField(QString file_);
   
   /// \brief draws robot information, ball information and obstacle
   /// information based on info on robot_info, in worldModel matrix
   void drawWorldModel();
   
   /// \brief sets new information to holding robotInfo structure of
   /// the class, the robot_info object
   /// \param msg - new information to be set
   void setRobotInfo(robotInfo msg);
   
      
   /// \brief returns absolute file path for field file in config folder
   /// \param field_name - name of the field file
   /// \return - absolute file path of field file
   QString getFieldFileName(std::string field_name);
private slots:
   /// \brief converts a point in meters in point in pixels in the
   /// viewport
   /// \param pos - point im meter coordinates to be converted to pixels
   /// \return point in pixels (viewport) converted from point in meters
   Point world2WorldModel(Point2d pos);
   
   /// \brief initializes absolute paths from local paths, to access
   /// configuration files
   void configFilePaths();
private:
   /// \brief image containers to draw the world model. field has the image
   /// of the field, which is only drawn once. worldModle is a copy of field
   /// and info from robot_info is added. relayWModel is a copy of worldModel
   Mat field,worldModel,relayWModel;
   /// \brief structure containing diverse information/dimensions from a field
   fieldDimensions fieldAnatomy;
   /// \brief File Paths
   QString mainFilePath, configFolderPath;
   QString fieldPath;
   /// \brief robotInfo message object, that holds world model information
   robotInfo robot_info;
   /// \brief id of the robot to be visualized
   int robot_id;
};

#endif // VISUALIZER_H
