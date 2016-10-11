#include "imagecalibrator.h"

//Constructor of the class
ImageCalibrator::ImageCalibrator()
{
   variablesInitialization();
}

// Miscellaneous variables Initialization
// TODO :: Use file for variables initialization
void ImageCalibrator::variablesInitialization()
{
    processed = Mat(480,480,CV_8UC3,Scalar(0,0,0));
    pix = Mat(1,1,CV_8UC3,Scalar(0,0,0));
    double morph_size = 1.5;
    element = getStructuringElement(2, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
}

// Converts rgb struct into hsv values, returns hsv struct
// TODO :: Rewrite this function
hsv ImageCalibrator::rgbtohsv(rgb in)
{
    hsv temp;
    /*pix.data[0] = in.r; pix.data[1] = in.g; pix.data[2] = in.b;
    cvtColor(pix,pix,CV_BGR2HSV);
    temp.h = pix.data[0]; temp.s = pix.data[1]; temp.v = pix.data[2];*/
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
// Returns binary image of the buffer, given YUV(or HSV) ranges
void ImageCalibrator::getBinary(Mat *in, labelConfiguration labelconf)
{
    //Returns binary representation of a certain range
    Vec3b *pixel; // iterator to run through captured image
    rgb pix; hsv pix2;

    for(int i = 0; i < 480; ++i){
        pixel = in->ptr<Vec3b>(i);
        for (int j = 0; j<480; ++j){
            pix.r = pixel[j][2]; pix.g = pixel[j][1]; pix.b = pixel[j][0];
            pix2 = rgbtohsv(pix);

            if((pix2.h>=labelconf.lb_calib[0][0])&&(pix2.h<=labelconf.lb_calib[0][1]) && (pix2.s>=labelconf.lb_calib[1][0])&&(pix2.s<=labelconf.lb_calib[1][1]) && (pix2.v>=labelconf.lb_calib[2][0])&&(pix2.v<=labelconf.lb_calib[2][1]))
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

void ImageCalibrator::updateCurrentConfiguration(LABEL_t label, COMPONENT_t component, RANGE_t range, int value)
{
   lutconfig.lut_calib[label].lb_calib[component][range] = value; 
}

labelConfiguration ImageCalibrator::getLabelConfiguration(LABEL_t label)
{
   return lutconfig.lut_calib[label];
}
