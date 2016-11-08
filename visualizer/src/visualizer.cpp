#include "visualizer.h"

/// \brief constructor of the class. Calls initialization functions.
/// Calls file path initialization, calling also the function that
/// draws the initial field.
/// \param id - Id of the robot that will be visualized
Visualizer::Visualizer(int id, QObject *parent) : QObject(parent)
{
   robot_id = id;
   configFilePaths();
   initField(fieldPath);
   field.copyTo(relayWModel);
}

/// \brief function that returns a pointer to the current image 
/// representing the world model described by the robot
/// \return pointer to the matrix realWModel
Mat *Visualizer::getWorldModel()
{
   return &relayWModel;
}
  
/// \brief initializes field image given the field dimensions and
/// data in a file.
/// \param file_ - path of the file containing field dimensions
void Visualizer::initField(QString file_)
{
    // Create view of current Field
    QFile file(file_);
    if(!file.open(QIODevice::ReadOnly)) {
        ROS_ERROR("Error reading field.view.");
        exit(6);
    }
    QTextStream in(&file);

    QString value; int counter = 0;
    while(!in.atEnd()){
       value = in.readLine();
       fieldAnatomy.dimensions[counter] = value.right(value.size()-value.indexOf('=')-1).toInt();
       counter++;
    }

    //using 1px -> FACTORcm relation
    int relation = fieldAnatomy.fieldDims.FACTOR*10;
    int anchorPoint1 = 0,anchorPoint2 = 0;
    for(int i=0;i<counter;i++) if(i!=18) fieldAnatomy.dimensions[i] /= relation;
    field = Mat(fieldAnatomy.fieldDims.TOTAL_WIDTH,fieldAnatomy.fieldDims.TOTAL_LENGTH,CV_8UC3,Scalar(0,120,0));
    //draw outter line
    anchorPoint1 = (fieldAnatomy.fieldDims.TOTAL_LENGTH-fieldAnatomy.fieldDims.LENGTH)/2+fieldAnatomy.fieldDims.LINE_WIDTH/2;
    anchorPoint2 = (fieldAnatomy.fieldDims.TOTAL_WIDTH-fieldAnatomy.fieldDims.WIDTH)/2+fieldAnatomy.fieldDims.LINE_WIDTH/2;
    rectangle(field,Point(anchorPoint1,anchorPoint2),Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH
            ,anchorPoint2+fieldAnatomy.fieldDims.WIDTH),Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
    //draw center line and circles
    line(field,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH/2,anchorPoint2)
        ,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH/2,anchorPoint2+fieldAnatomy.fieldDims.WIDTH),
        Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
    circle(field,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH/2,anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2),
           fieldAnatomy.fieldDims.SPOT_CENTER+2,Scalar(255,255,255),-1);
    circle(field,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH/2,anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2),
           fieldAnatomy.fieldDims.CENTER_RADIUS,Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
    //draw left area
    rectangle(field,Point(anchorPoint1,anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2-fieldAnatomy.fieldDims.AREA_WIDTH1/2)
            ,Point(anchorPoint1+fieldAnatomy.fieldDims.AREA_LENGTH1,anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2+fieldAnatomy.fieldDims.AREA_WIDTH1/2)
            ,Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
    rectangle(field,Point(anchorPoint1,anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2-fieldAnatomy.fieldDims.AREA_WIDTH2/2)
            ,Point(anchorPoint1+fieldAnatomy.fieldDims.AREA_LENGTH2,anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2+fieldAnatomy.fieldDims.AREA_WIDTH2/2)
            ,Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
    circle(field,Point(anchorPoint1+fieldAnatomy.fieldDims.DISTANCE_PENALTY,anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2),
           fieldAnatomy.fieldDims.SPOT_CENTER,Scalar(255,255,255),-1);
    //draw right area
    rectangle(field,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH,anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2-fieldAnatomy.fieldDims.AREA_WIDTH1/2)
            ,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH-fieldAnatomy.fieldDims.AREA_LENGTH1,anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2+fieldAnatomy.fieldDims.AREA_WIDTH1/2)
            ,Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
    rectangle(field,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH,anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2-fieldAnatomy.fieldDims.AREA_WIDTH2/2)
            ,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH-fieldAnatomy.fieldDims.AREA_LENGTH2,anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2+fieldAnatomy.fieldDims.AREA_WIDTH2/2)
            ,Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
    circle(field,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH-fieldAnatomy.fieldDims.DISTANCE_PENALTY,anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2),
           fieldAnatomy.fieldDims.SPOT_CENTER,Scalar(255,255,255),-1);
    //draw left corners
    ellipse(field,Point(anchorPoint1,anchorPoint2),Size(fieldAnatomy.fieldDims.RADIUS_CORNER,fieldAnatomy.fieldDims.RADIUS_CORNER),
            0.0,0.0,90.0,Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
    ellipse(field,Point(anchorPoint1,anchorPoint2+fieldAnatomy.fieldDims.WIDTH),Size(fieldAnatomy.fieldDims.RADIUS_CORNER,fieldAnatomy.fieldDims.RADIUS_CORNER),
            0.0,270.0,360.0,Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
    //draw right corners
    ellipse(field,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH,anchorPoint2),Size(fieldAnatomy.fieldDims.RADIUS_CORNER,fieldAnatomy.fieldDims.RADIUS_CORNER),
            0.0,90.0,180.0,Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
    ellipse(field,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH,anchorPoint2+fieldAnatomy.fieldDims.WIDTH),Size(fieldAnatomy.fieldDims.RADIUS_CORNER,fieldAnatomy.fieldDims.RADIUS_CORNER),
            0.0,180.0,270.0,Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
    //draw ref box
    rectangle(field,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH/2-50,anchorPoint2+fieldAnatomy.fieldDims.WIDTH+5),
             Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH/2+50,anchorPoint2+fieldAnatomy.fieldDims.WIDTH+25),
              Scalar(255,0,255),-1);
    putText(field,"REFBOX",Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH/2-30,anchorPoint2+fieldAnatomy.fieldDims.WIDTH+20),
            CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(0,0,0),2);
    //draw right goalie
    rectangle(field,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH,anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2-fieldAnatomy.fieldDims.GOALIE_LENGTH/2-fieldAnatomy.fieldDims.GOALIE_POST_WIDTH/2)
            ,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH+fieldAnatomy.fieldDims.GOALIE_WIDTH,anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2+fieldAnatomy.fieldDims.GOALIE_LENGTH/2+fieldAnatomy.fieldDims.GOALIE_POST_WIDTH/2)
            ,Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
    //draw left goalie
    rectangle(field,Point(anchorPoint1,anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2-fieldAnatomy.fieldDims.GOALIE_LENGTH/2-fieldAnatomy.fieldDims.GOALIE_POST_WIDTH/2)
            ,Point(anchorPoint1-fieldAnatomy.fieldDims.GOALIE_WIDTH,anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2+fieldAnatomy.fieldDims.GOALIE_LENGTH/2+fieldAnatomy.fieldDims.GOALIE_POST_WIDTH/2)
            ,Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);

}

/// \brief draws robot information, ball information and obstacle
/// information based on info on robot_info, in worldModel matrix
void Visualizer::drawWorldModel()
{
   field.copyTo(worldModel);
   float velModule = 0.0;
   int sizeLine = 0;
   float direction = 0.0;
   Point arrow;
   
   //Draw Obstacles
   //##############################
   for(unsigned int i= 0;i < robot_info.obstacles.size(); i++){
      circle(worldModel,world2WorldModel(Point2d(robot_info.obstacles[i].x,robot_info.obstacles[i].y)),fieldAnatomy.fieldDims.ROBOT_DIAMETER/3,Scalar(0,0,0),-1);
   }
   //##############################
   
   //Draw Robot and Velocity vector
   //##############################
   Point robot = world2WorldModel(Point2d(robot_info.robot_pose.x,robot_info.robot_pose.y));
   ellipse(worldModel,robot,Size(fieldAnatomy.fieldDims.ROBOT_DIAMETER/2+1,fieldAnatomy.fieldDims.ROBOT_DIAMETER/2+1),
         robot_info.robot_pose.z+90.0,45.0,315.0,Scalar(255,255,0),-1);
         
   velModule = sqrt((robot_info.robot_velocity.x)*(robot_info.robot_velocity.x)+(robot_info.robot_velocity.y)*(robot_info.robot_velocity.y));
   sizeLine = velModule*10;
   direction = atan2(robot_info.robot_velocity.y,robot_info.robot_velocity.x);
   if(sizeLine<100 && sizeLine>0){
      arrow.x = robot.x+cos(direction)*sizeLine; 
      arrow.y = robot.y+sin(direction)*sizeLine;
      line(worldModel,robot,arrow,Scalar(255,0,0),3);
   }
   
   QString txt = "Visualizer for Robot ";
   txt += QString::number(robot_id);
   txt += "   [KEYS] ESC - Close | I - Interest Points";
   putText(worldModel,txt.toStdString(),Point(50,20),FONT_HERSHEY_SIMPLEX,0.45,Scalar(0,0,0),2);
   //##############################
         
   //Draw Ball and Velocity vector
   //##############################
   if(robot_info.sees_ball){
      Point ball = world2WorldModel(Point2d(robot_info.ball_position.x,robot_info.ball_position.y));
      circle(worldModel,ball,11/(fieldAnatomy.fieldDims.FACTOR),Scalar(0,127,255),-1);
     
      if(robot_info.has_ball){
         circle(worldModel,ball,11/(fieldAnatomy.fieldDims.FACTOR),Scalar(0,0,255),1);   
      }
      
      velModule = sqrt((robot_info.ball_velocity.x)*(robot_info.ball_velocity.x)+(robot_info.ball_velocity.y)*(robot_info.ball_velocity.y));
      sizeLine = velModule*10;
      direction = atan2(robot_info.ball_velocity.y,robot_info.ball_velocity.x);
      if(sizeLine<100 && sizeLine>0){
	      arrow.x = ball.x+cos(direction)*sizeLine; 
	      arrow.y = ball.y+sin(direction)*sizeLine;
	      line(worldModel,ball,arrow,Scalar(0,255,255),2);
      }
   }
   
   //Draw interest Points
   for(unsigned int i=0;i<robot_info.interest_points.size();i++){
      Scalar color;
      switch(robot_info.interest_points[i].type){
         case 0:{ // LINE POINTS
            color = Scalar(255,0,0);
            break;
         }
         case 1:{ // BALL POINTS
            color = Scalar(0,0,255);
            break;
         }
         case 2:{ // OBSTACLE POINTS
            color = Scalar(0,0,0);
            break;
         }
      }
      
      circle(worldModel, world2WorldModel(Point2d(robot_info.interest_points[i].pos.x,
      robot_info.interest_points[i].pos.y)),2,color,-1);
   }
   //##############################
   worldModel.copyTo(relayWModel);
}

/// \brief sets new information to holding robotInfo structure of
/// the class, the robot_info object
/// \param msg - new information to be set
void Visualizer::setRobotInfo(robotInfo msg)
{
   robot_info = msg;
}

/// \brief converts a point in meters in point in pixels in the
/// viewport
/// \param pos - point im meter coordinates to be converted to pixels
/// \return point in pixels (viewport) converted from point in meters
Point Visualizer::world2WorldModel(Point2d pos)
{
   Point converted = Point(0,0);
   double totalLength = fieldAnatomy.fieldDims.TOTAL_LENGTH*fieldAnatomy.fieldDims.FACTOR/100.0;
   double totalWidth = fieldAnatomy.fieldDims.TOTAL_WIDTH*fieldAnatomy.fieldDims.FACTOR/100.0;

   converted.x = pos.x*(fieldAnatomy.fieldDims.TOTAL_LENGTH/totalLength)+fieldAnatomy.fieldDims.TOTAL_LENGTH/2+
         fieldAnatomy.fieldDims.LINE_WIDTH/2;
   converted.y = pos.y*(fieldAnatomy.fieldDims.TOTAL_WIDTH/totalWidth)+fieldAnatomy.fieldDims.TOTAL_WIDTH/2+
         fieldAnatomy.fieldDims.LINE_WIDTH/2;
   return converted;
}

/// \brief initializes absolute paths from local paths, to access
/// configuration files
void Visualizer::configFilePaths()
{
   QString home = getenv("HOME");
   mainFilePath = home+"/"+QString(mainPath);
   configFolderPath = home+"/"+QString(cfgFolderPath);

   QFile file(mainFilePath);
   if(!file.open(QIODevice::ReadOnly)) {
      ROS_ERROR("Error reading main.cfg in %s",mainFilePath.toStdString().c_str());
      exit(7);
   }
   QTextStream in(&file);

   QString fieldID = in.readLine();// FieldName
   fieldPath = configFolderPath+fieldID+".view";
   file.close();
}
