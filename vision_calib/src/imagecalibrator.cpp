#include "imagecalibrator.h"

/// \brief class constructor. Calls variablesInitializtion function
ImageCalibrator::ImageCalibrator()
{
   variablesInitialization();
}

/// \brief function to initialize data. Creates Mats and morphing elements
void ImageCalibrator::variablesInitialization()
{
    processed = Mat(480,480,CV_8UC3,Scalar(0,0,0));
    double morph_size = 1.5;
    element = getStructuringElement(2, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
}

/// \brief function to convert an rgb pixel into hsv in 0-180/0-255/0-255 range.
/// \params [in] : in -> rgb struct containing rgb values to be converted
/// \params [out] : hsv struct containing hsv values resultant from the conversion
hsv ImageCalibrator::rgbtohsv(rgb in)
{
    hsv temp;
    int min = 0, max = 0, delta = 0;
    if(in.r<in.g)min=in.r; else min=in.g;
    if(in.b<min)min=in.b;

    if(in.r>in.g)max=in.r; else max=in.g;
    if(in.b>max)max=in.b;

    temp.v = max;                // v, 0..255
    delta = max - min;                      // 0..255, < v

    if(max != 0)
        temp.s = (int)(delta)*255/max;        // s, 0..255
    else {
        // r = g = b = 0        // s = 0, v is undefined
        temp.s = 0;
        temp.h = 0;
        return temp;
    }
    if(delta==0) temp.h = 0;
    else {
        if( in.r == max )
            temp.h = (in.g - in.b)*30/delta;        // between yellow & magenta
        else if( in.g == max )
            temp.h = 60 + (in.b - in.r)*30/delta;    // between cyan & yellow
        else
            temp.h = 120 + (in.r - in.g)*30/delta;    // between magenta & cyan

        while( temp.h < 0 ) temp.h += 180;
    }

    if(temp.h>160){ //wrap around
        temp.h = (int)(-0.11111*temp.h)+20;
    }
    return temp;
}

/// \brief function to return binary representation of an image given a certain
/// hsv configuration (range).
/// \params [in] : in -> pointer to target image (in and out image container)
///                labelconf -> hsv configuration for a certain label
void ImageCalibrator::getBinary(Mat *in, minho_team_ros::label labelconf)
{
    //Returns binary representation of a certain range
    Vec3b *pixel; // iterator to run through captured image
    rgb pix; hsv pix2;

    for(int i = 0; i < 480; ++i){
        pixel = in->ptr<Vec3b>(i);
        for (int j = 0; j<480; ++j){
            pix.r = pixel[j][2]; pix.g = pixel[j][1]; pix.b = pixel[j][0];
            pix2 = rgbtohsv(pix);

            if((pix2.h>=labelconf.H.min)&&(pix2.h<=labelconf.H.max) &&
            (pix2.s>=labelconf.S.min)&&(pix2.s<=labelconf.S.max) &&
            (pix2.v>=labelconf.V.min)&&(pix2.v<=labelconf.V.max))
            {
                pixel[j][2] = 255;
                pixel[j][1] = 255;
                pixel[j][0] = 255;
            }else {
                pixel[j][2] = 0;
                pixel[j][1] = 0;
                pixel[j][0] = 0;
            }
        }
    }
}

/// \brief function to update a certain range value in lutconfig message.
/// \params [in] : label -> label to be changed
///                component -> component of label (h,s or v) to be changed
///                range -> range of component to be changed (min or max)
///                value -> value to be applied to range
void ImageCalibrator::updateCurrentConfiguration(LABEL_t label, COMPONENT_t component, RANGE_t range, int value)
{
   minho_team_ros::label *lb;
   minho_team_ros::range *comp;
   if(label==FIELD) lb = &lutconfig.field;
   else if(label==LINE) lb = &lutconfig.line;
   else if(label==BALL) lb = &lutconfig.ball;
   else if(label==OBSTACLE) lb = &lutconfig.obstacle;
   else return;
   
   if(component==H) comp = &lb->H;
   else if(component==S) comp = &lb->S;
   else if(component==V) comp = &lb->V;
   else return;
   
   if(range==MIN) comp->min = value;
   else if(range==MAX) comp->max = value;
   else return;
   
}

/// \brief function to get current label configuration
/// \params [in] : label -> label's id to be get
/// \params [out] : label -> label's configuration
minho_team_ros::label ImageCalibrator::getLabelConfiguration(LABEL_t label)
{
   if(label==FIELD) return lutconfig.field;
   else if(label==LINE) return lutconfig.line;
   else if(label==BALL) return lutconfig.ball;
   else return lutconfig.obstacle;
}

/// \brief function to get current visionHSVConfig configuration
/// \params [out] : visionHSVConfig -> current visionHSVConfig configuration
minho_team_ros::visionHSVConfig ImageCalibrator::getLutConfiguration()
{
   return lutconfig;
}

/// \brief function to get current imageConfig configuration
/// \params [out] : imageConfig -> current imageConfig configuration
minho_team_ros::imageConfig ImageCalibrator::getImageConfiguration()
{
   return imageConf;
}

/// \brief applies a configuration for lutconfig giver a visionHSVConfig message
/// \params [in] : msg -> visionHSVConfig message to be used 
void ImageCalibrator::lutConfigFromMsg(visionHSVConfig msg)
{
   lutconfig = msg;     
}

/// \brief applies a configuration for mirrorConf giver a mirrorConfig message
/// \params [in] : msg -> mirrorConfig message to be used 
void ImageCalibrator::mirrorConfigFromMsg(mirrorConfig msg)
{
   mirrorConf = msg;
   generateDistanceVectors();
}

/// \brief applies a configuration for imageConf giver a imageConfig message
/// \params [in] : msg -> imageConfig message to be used 
void ImageCalibrator::imageConfigFromMsg(imageConfig msg)
{
   imageConf = msg;
   generateDistanceLookUpTable();
}

/// \brief function to draw a center point and a crosshair given the current
/// imageConf configuration message
/// \params [in] : image -> target image container where things will be drawn
void ImageCalibrator::drawCenter(Mat *image)
{
   circle(*image,Point(imageConf.center_x,imageConf.center_y),3,Scalar(0,0,255),-1);
   int hlen = 400;
   line(*image,Point(-cos(imageConf.tilt*TO_RAD)*hlen+imageConf.center_x,-sin(imageConf.tilt*TO_RAD)*hlen+imageConf.center_y),
   Point(cos(imageConf.tilt*TO_RAD)*hlen+imageConf.center_x,sin(imageConf.tilt*TO_RAD)*hlen+imageConf.center_y),
   Scalar(255,0,0),1);
   line(*image,Point(-cos(imageConf.tilt*TO_RAD+M_PI_2)*hlen+imageConf.center_x,-sin(imageConf.tilt*TO_RAD+M_PI_2)*hlen+imageConf.center_y),
   Point(cos(imageConf.tilt*TO_RAD+M_PI_2)*hlen+imageConf.center_x,sin(imageConf.tilt*TO_RAD+M_PI_2)*hlen+imageConf.center_y),
   Scalar(255,0,0),1);
}

/// \brief function to generate new distLookUpTable based on current mirrorConfig
void ImageCalibrator::generateDistanceLookUpTable()
{
   distLookUpTable.clear();
   distLookUpTable = vector<vector<Point2d> >(480*480,vector<Point2d>(0));

   for(int i=0; i<=480; i++){ // columns
     for(int j=0; j<=480; j++){ // rows
         double dist = d2pWorld(d2p(Point(imageConf.center_x,imageConf.center_y),Point(j,i)));
         double angulo = (atan2((j-imageConf.center_x),(i-imageConf.center_y))*(180.0/M_PI))-imageConf.tilt;
         while(angulo<0.0)angulo+=360.0;
         while(angulo>360.0)angulo-=360.0;
         distLookUpTable[j].push_back(Point2d(dist,angulo));
     }
   }    
}

/// \brief function to generate new distPix and distReal based on current mirrorConfig
void ImageCalibrator::generateDistanceVectors()
{
   distReal.clear(); distPix.clear();
   for(float dist=mirrorConf.step;dist<=mirrorConf.max_distance;dist+=mirrorConf.step) 
      distReal.push_back(dist);
   for(int i=0;i<mirrorConf.pixel_distances.size();i++) 
      distPix.push_back((double)mirrorConf.pixel_distances[i]);
}

/// \brief function to convert pixels to meters based on distPix and distReal
/// \params [in] : pixels -> distance in pixels to be converted to meters
double ImageCalibrator::d2pWorld(int pixels)
{
   unsigned int index = 0;
   while(pixels>distPix[index] && index<(distPix.size()-1))index++;
   if(index<=0)return 0;
   if(pixels>distPix[distPix.size()-1]){
      return -1;
   }
   else return distReal[index-1]+(((pixels-distPix[index-1])*(distReal[index]-distReal[index-1]))/(distPix[index]-distPix[index-1]));
}

/// \brief computes euclidean distance between points p1 and p2
/// \params [in] : p1 -> first point in distance calculation
///                p2 -> second point in distance calculation
///         [out] : int -> euclidean distance between p1 and p2
int ImageCalibrator::d2p(Point p1,Point p2)
{
   return sqrt(pow(p2.x-p1.x,2)+pow(p2.y-p1.y,2));
}

/// \brief function to map point in image (pixels) to point in world (meters)
/// \params [in] : p -> point to be mapped (pixels)
///         [out] : Point2d -> point mapped in meters
Point2d ImageCalibrator::worldMapping(Point p)
{
   if(p.x<0 || p.x>=480 || p.y<0 || p.y>=480) return Point2d(0.0,0.0);
   return Point2d(distLookUpTable[p.x][p.y].x,distLookUpTable[p.x][p.y].y);
}
