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
//using 1px -> FACTORcm relation
      int anchorPoint1 = 0,anchorPoint2 = 0;
      for(int i=0;i<23;i++) if(i!=18) fieldAnatomy.dimensions[i] /= relation;
      field = Mat(fieldAnatomy.fieldDims.TOTAL_WIDTH,fieldAnatomy.fieldDims.TOTAL_LENGTH,CV_8UC3,Scalar(0,120,0));
      //draw outter line
      anchorPoint1 = (fieldAnatomy.fieldDims.TOTAL_LENGTH-fieldAnatomy.fieldDims.LENGTH)/2+fieldAnatomy.fieldDims.LINE_WIDTH/2;
      anchorPoint2 = (fieldAnatomy.fieldDims.TOTAL_WIDTH-fieldAnatomy.fieldDims.WIDTH)/2+fieldAnatomy.fieldDims.LINE_WIDTH/2;
      rectangle(field,Rect(Point(anchorPoint1,anchorPoint2),
             Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH-fieldAnatomy.fieldDims.LINE_WIDTH,
                   anchorPoint2+fieldAnatomy.fieldDims.WIDTH-fieldAnatomy.fieldDims.LINE_WIDTH))
             ,Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
      //draw center line and circles
      line(field,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH/2-fieldAnatomy.fieldDims.LINE_WIDTH/2,anchorPoint2)
         ,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH/2-fieldAnatomy.fieldDims.LINE_WIDTH/2,
         anchorPoint2+fieldAnatomy.fieldDims.WIDTH-fieldAnatomy.fieldDims.LINE_WIDTH),
         Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
      circle(field,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH/2-fieldAnatomy.fieldDims.LINE_WIDTH/2,
                        anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2-fieldAnatomy.fieldDims.LINE_WIDTH/2),
            fieldAnatomy.fieldDims.SPOT_CENTER,Scalar(255,255,255),-1);

      circle(field,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH/2-fieldAnatomy.fieldDims.LINE_WIDTH/2,
                        anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2-fieldAnatomy.fieldDims.LINE_WIDTH/2),
            fieldAnatomy.fieldDims.CENTER_RADIUS-fieldAnatomy.fieldDims.LINE_WIDTH/2,Scalar(255,255,255),
            fieldAnatomy.fieldDims.LINE_WIDTH);
      //draw ref box
      rectangle(field,Point(fieldAnatomy.fieldDims.TOTAL_LENGTH/2-500/relation,anchorPoint2+fieldAnatomy.fieldDims.WIDTH+50/relation),
              Point(fieldAnatomy.fieldDims.TOTAL_LENGTH/2+500/relation,anchorPoint2+fieldAnatomy.fieldDims.WIDTH+250/relation),
               Scalar(255,0,255),-1);
      putText(field,"REFBOX",Point(fieldAnatomy.fieldDims.TOTAL_LENGTH/2-30,anchorPoint2+fieldAnatomy.fieldDims.WIDTH+20),
             CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(0,0,0),2);
      //draw left corners
      ellipse(field,Point(anchorPoint1,anchorPoint2),Size(fieldAnatomy.fieldDims.RADIUS_CORNER-fieldAnatomy.fieldDims.LINE_WIDTH
             ,fieldAnatomy.fieldDims.RADIUS_CORNER-fieldAnatomy.fieldDims.LINE_WIDTH),
             0.0,0.0,90.0,Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
      ellipse(field,Point(anchorPoint1,anchorPoint2+fieldAnatomy.fieldDims.WIDTH-fieldAnatomy.fieldDims.LINE_WIDTH),
             Size(fieldAnatomy.fieldDims.RADIUS_CORNER-fieldAnatomy.fieldDims.LINE_WIDTH
             ,fieldAnatomy.fieldDims.RADIUS_CORNER-fieldAnatomy.fieldDims.LINE_WIDTH),
             0.0,270.0,360.0,Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
      //draw right corners
      ellipse(field,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH-fieldAnatomy.fieldDims.LINE_WIDTH,anchorPoint2),
             Size(fieldAnatomy.fieldDims.RADIUS_CORNER-fieldAnatomy.fieldDims.LINE_WIDTH
             ,fieldAnatomy.fieldDims.RADIUS_CORNER-fieldAnatomy.fieldDims.LINE_WIDTH),
             0.0,90.0,180.0,Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
      ellipse(field,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH-fieldAnatomy.fieldDims.LINE_WIDTH,anchorPoint2+fieldAnatomy.fieldDims.WIDTH-fieldAnatomy.fieldDims.LINE_WIDTH),
             Size(fieldAnatomy.fieldDims.RADIUS_CORNER-fieldAnatomy.fieldDims.LINE_WIDTH
             ,fieldAnatomy.fieldDims.RADIUS_CORNER-fieldAnatomy.fieldDims.LINE_WIDTH),
             0.0,180.0,270.0,Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
      //draw left goalie
      rectangle(field,Point(anchorPoint1,fieldAnatomy.fieldDims.TOTAL_WIDTH/2-fieldAnatomy.fieldDims.GOALIE_LENGTH/2+fieldAnatomy.fieldDims.GOALIE_POST_WIDTH/2)
             ,Point(anchorPoint1-fieldAnatomy.fieldDims.GOALIE_WIDTH,fieldAnatomy.fieldDims.TOTAL_WIDTH/2+fieldAnatomy.fieldDims.GOALIE_LENGTH/2-fieldAnatomy.fieldDims.GOALIE_POST_WIDTH/2)
             ,Scalar(255,0,0),fieldAnatomy.fieldDims.LINE_WIDTH);
      //draw right goalie
      rectangle(field,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH-fieldAnatomy.fieldDims.LINE_WIDTH-1,
              fieldAnatomy.fieldDims.TOTAL_WIDTH/2-fieldAnatomy.fieldDims.GOALIE_LENGTH/2+fieldAnatomy.fieldDims.GOALIE_POST_WIDTH/2)
             ,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH-fieldAnatomy.fieldDims.LINE_WIDTH+fieldAnatomy.fieldDims.GOALIE_WIDTH,
             fieldAnatomy.fieldDims.TOTAL_WIDTH/2+fieldAnatomy.fieldDims.GOALIE_LENGTH/2-fieldAnatomy.fieldDims.GOALIE_POST_WIDTH/2)
             ,Scalar(255,0,0),fieldAnatomy.fieldDims.LINE_WIDTH);
      /*line(field,Point(0,fieldAnatomy.fieldDims.TOTAL_WIDTH/2),
          Point(fieldAnatomy.fieldDims.TOTAL_LENGTH,fieldAnatomy.fieldDims.TOTAL_WIDTH/2),Scalar(255,255,255));*/
      //draw left area
      rectangle(field,Point(anchorPoint1,fieldAnatomy.fieldDims.TOTAL_WIDTH/2-fieldAnatomy.fieldDims.AREA_WIDTH1/2+fieldAnatomy.fieldDims.LINE_WIDTH/2)
             ,Point(anchorPoint1+fieldAnatomy.fieldDims.AREA_LENGTH1-fieldAnatomy.fieldDims.LINE_WIDTH,
                    fieldAnatomy.fieldDims.TOTAL_WIDTH/2+fieldAnatomy.fieldDims.AREA_WIDTH1/2-fieldAnatomy.fieldDims.LINE_WIDTH/2)
             ,Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
      rectangle(field,Point(anchorPoint1,fieldAnatomy.fieldDims.TOTAL_WIDTH/2-fieldAnatomy.fieldDims.AREA_WIDTH2/2+fieldAnatomy.fieldDims.LINE_WIDTH/2)
             ,Point(anchorPoint1+fieldAnatomy.fieldDims.AREA_LENGTH2-fieldAnatomy.fieldDims.LINE_WIDTH,
                    fieldAnatomy.fieldDims.TOTAL_WIDTH/2+fieldAnatomy.fieldDims.AREA_WIDTH2/2-fieldAnatomy.fieldDims.LINE_WIDTH/2)
             ,Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
      circle(field,Point(anchorPoint1+fieldAnatomy.fieldDims.DISTANCE_PENALTY-fieldAnatomy.fieldDims.SPOTS/2-fieldAnatomy.fieldDims.LINE_WIDTH,
            anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2-fieldAnatomy.fieldDims.LINE_WIDTH/2),
            fieldAnatomy.fieldDims.SPOTS,Scalar(255,255,255),-1);
      //draw right area
      rectangle(field,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH-fieldAnatomy.fieldDims.LINE_WIDTH-1
                     ,fieldAnatomy.fieldDims.TOTAL_WIDTH/2-fieldAnatomy.fieldDims.AREA_WIDTH1/2+fieldAnatomy.fieldDims.LINE_WIDTH/2)
             ,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH-fieldAnatomy.fieldDims.AREA_LENGTH1,anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2+fieldAnatomy.fieldDims.AREA_WIDTH1/2)
             ,Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
      rectangle(field,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH-fieldAnatomy.fieldDims.LINE_WIDTH-1
                     ,fieldAnatomy.fieldDims.TOTAL_WIDTH/2-fieldAnatomy.fieldDims.AREA_WIDTH2/2+fieldAnatomy.fieldDims.LINE_WIDTH/2)
             ,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH-fieldAnatomy.fieldDims.AREA_LENGTH2,anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2+fieldAnatomy.fieldDims.AREA_WIDTH2/2)
             ,Scalar(255,255,255),fieldAnatomy.fieldDims.LINE_WIDTH);
      circle(field,Point(anchorPoint1+fieldAnatomy.fieldDims.LENGTH+fieldAnatomy.fieldDims.LINE_WIDTH/2-fieldAnatomy.fieldDims.DISTANCE_PENALTY,
            anchorPoint2+fieldAnatomy.fieldDims.WIDTH/2-fieldAnatomy.fieldDims.LINE_WIDTH/2),
            fieldAnatomy.fieldDims.SPOTS,Scalar(255,255,255),-1);

}

/// \brief draws robot information, ball information and obstacle
/// information based on info on robot_info, in worldModel matrix
void Visualizer::drawWorldModel(bool drawKeeperInfo)
{
   field.copyTo(worldModel);
   float velModule = 0.0;
   int sizeLine = 0;
   float direction = 0.0;
   Point arrow;

   for(unsigned int i=0; i<path_points.voronoi_seg.size(); i++) {
      line(worldModel,world2WorldModel(Point2d(path_points.voronoi_seg[i].ini.x,path_points.voronoi_seg[i].ini.y)),
                        world2WorldModel(Point2d(path_points.voronoi_seg[i].fini.x,path_points.voronoi_seg[i].fini.y)),
                        Scalar(0,255,255), 1);
   }

   for(unsigned int i=0; i<path_points.intersect_seg.size(); i++) {
      line(worldModel,world2WorldModel(Point2d(path_points.intersect_seg[i].ini.x,path_points.intersect_seg[i].ini.y)),
                        world2WorldModel(Point2d(path_points.intersect_seg[i].fini.x,path_points.intersect_seg[i].fini.y)),
                        Scalar(160,160,160), 1);
   }

   if(path_points.dijk_path.size() > 1) {
      for(unsigned int i=0; i<path_points.dijk_path.size()-1; i++) {
      line(worldModel,world2WorldModel(Point2d(path_points.dijk_path[i].x,path_points.dijk_path[i].y)),
                      world2WorldModel(Point2d(path_points.dijk_path[i+1].x,path_points.dijk_path[i+1].y)),
                      Scalar(255,0,0), 3);
      }
   }

   if(path_points.dijk_path_obst_circle.size() > 1) {
      for(unsigned int i=0; i<path_points.dijk_path_obst_circle.size()-1; i++) {
      line(worldModel,world2WorldModel(Point2d(path_points.dijk_path_obst_circle[i].x,path_points.dijk_path_obst_circle[i].y)),
                      world2WorldModel(Point2d(path_points.dijk_path_obst_circle[i+1].x,path_points.dijk_path_obst_circle[i+1].y)),
                      Scalar(255,153,51), 2);
      }
   }

   if(path_points.smooth_path.size() > 1) {
      for(unsigned int i=0; i<path_points.smooth_path.size()-1; i++) {
      line(worldModel,world2WorldModel(Point2d(path_points.smooth_path[i].x,path_points.smooth_path[i].y)),
                      world2WorldModel(Point2d(path_points.smooth_path[i+1].x,path_points.smooth_path[i+1].y)),
                      Scalar(0,76,153), 3);
      }
   }

   if(path_points.smooth_path_obst_circle.size() > 1) {
      for(unsigned int i=0; i<path_points.smooth_path_obst_circle.size()-1; i++) {
      line(worldModel,world2WorldModel(Point2d(path_points.smooth_path_obst_circle[i].x,path_points.smooth_path_obst_circle[i].y)),
                      world2WorldModel(Point2d(path_points.smooth_path_obst_circle[i+1].x,path_points.smooth_path_obst_circle[i+1].y)),
                      Scalar(51,153,255), 2);
      }
   }

   if(path_points.path.size() > 1) {
      for(unsigned int i=0; i<path_points.path.size()-1; i++) {
      line(worldModel,world2WorldModel(Point2d(path_points.path[i].x,path_points.path[i].y)),
                      world2WorldModel(Point2d(path_points.path[i+1].x,path_points.path[i+1].y)),
                      Scalar(0,0,255), 3);
      }
   }


   //Draw Obstacles
   //##############################
   for(unsigned int i= 0;i < robot_info.obstacles.size(); i++){
      circle(worldModel,world2WorldModel(Point2d(robot_info.obstacles[i].x,robot_info.obstacles[i].y)),fieldAnatomy.fieldDims.ROBOT_DIAMETER/2+1,Scalar(0,0,0),-1);
   }

   //Draw Obstacles Circle
   //##############################
   for(unsigned int i= 0; i<path_points.obstacles_circle.size(); i=i+3) {
      circle(worldModel,world2WorldModel(Point2d(path_points.obstacles_circle[i], path_points.obstacles_circle[i+1])),
              (int)((path_points.obstacles_circle[i+2]*100.0)/2.0), Scalar(0,0,0), 1);
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
     
   if(drawKeeperInfo){    
      Point ball = world2WorldModel(Point2d(keeper_info.impact_zone.x,keeper_info.impact_zone.y));
         circle(worldModel,ball,20/(fieldAnatomy.fieldDims.FACTOR),Scalar(255,0,255),-1);
   }
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

void Visualizer::setGoalKeeperInfo(goalKeeperInfo msg)
{
   keeper_info = msg;
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
   QString home = QString::fromStdString(getenv("HOME"));
   QString commonDir = home+QString(COMMON_PATH);
   mainFilePath = commonDir+QString(MAINFILENAME);
   configFolderPath = commonDir+QString(FIELDS_PATH);

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

/// \brief returns absolute file path for field file in config folder
/// \param field_name - name of the field file
/// \return - absolute file path of field file
QString Visualizer::getFieldFileName(std::string field_name)
{
   return configFolderPath+QString(field_name.c_str())+".view";
}
