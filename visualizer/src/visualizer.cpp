#include "visualizer.h"

Visualizer::Visualizer(QObject *parent) : QObject(parent)
{
    configFilePaths();
    initField(fieldPath);
    field.copyTo(relayWModel);
}

Mat *Visualizer::getWorldModel()
{
	return &relayWModel;
}
    
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

void Visualizer::setRobotPosition(pose r_pose)
{
	robot_pose = r_pose;
}

void Visualizer::setBallPosition2D(position ball_pos)
{
	ball2DPosition = ball_pos;
}

void Visualizer::drawWorldModel()
{
    field.copyTo(worldModel);

    //draw robot
    Point robot = world2WorldModel(Point2d(robot_pose.x,robot_pose.y));
    ellipse(worldModel,robot,Size(fieldAnatomy.fieldDims.ROBOT_DIAMETER/2+1,fieldAnatomy.fieldDims.ROBOT_DIAMETER/2+1),
            robot_pose.z+90.0,45.0,315.0,Scalar(0,0,255),-1);
    
    circle(worldModel,world2WorldModel(Point2d(ball2DPosition.x,ball2DPosition.y))
    ,11/(fieldAnatomy.fieldDims.FACTOR),Scalar(0,127,255),-1);
    
    worldModel.copyTo(relayWModel);
}

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
