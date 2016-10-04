#include "imagecalibrator.h"

//Constructor of the class
ImageCalibrator::ImageCalibrator()
{
   resetLookUpTable();
   variablesInitialization();
}

// Miscellaneous variables Initialization
// TODO :: Use file for variables initialization
void ImageCalibrator::variablesInitialization()
{
    processed = Mat(480,480,CV_8UC3,Scalar(0,0,0));
    double morph_size = 1.5;
    element = getStructuringElement(2, Size( 2*morph_size + 1, 2*morph_size+1 ), Point( morph_size, morph_size ) );
}

// Paints a pixel accordingly to its classifier
void ImageCalibrator::paintPixel(int x, int y, int classifier,Mat *buf)
{
    switch(classifier){
        case UAV_NOCOLORS_BIT:{
            circle((*buf),Point(x,y),0.5,Scalar(127,127,127));
            break;
        }
        case UAV_WHITE_BIT:{
            circle((*buf),Point(x,y),0.5,Scalar(255,255,255));
            break;
        }
        case UAV_GREEN_BIT:{
            circle((*buf),Point(x,y),0.5,Scalar(0,255,0));
            break;
        }
        case UAV_ORANGE_BIT:{
            circle((*buf),Point(x,y),0.5,Scalar(0,0,255));
            break;
        }
        case UAV_BLACK_BIT:{
            circle((*buf),Point(x,y),0.5,Scalar(255,0,0));
            break;
        }
    }
}

// Converts rgb struct into hsv values, returns hsv struct
// TODO :: Rewrite this function
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

    if(temp.h>160){
        temp.h = (int)(-0.11111*temp.h)+20;
    }
    return temp;
}

// Returns the classifier of a pixel based on the Look Up Table
int ImageCalibrator::getClassifier(int x, int y, Mat *buffer)
{
    if(x<0 || x>=480 || y<0 || y>=480) return UAV_NOCOLORS_BIT;
    Vec3b *color = buffer->ptr<Vec3b>(y);
    long int index = (color[x][2]<<16) + (color[x][1]<<8) + (color[x][0]);
    return YUVLookUpTable[index];
}

// Returns binary image of the buffer, given YUV(or HSV) ranges
void ImageCalibrator::getBinary(Mat *in,int ymin,int ymax, int umin, int umax, int vmin, int vmax)
{
    //Returns binary representation of a certain range
    Vec3b *pixel; // iterator to run through captured image
    rgb pix; hsv pix2;

    for(int i = 0; i < 480; ++i){
        pixel = in->ptr<Vec3b>(i);
        for (int j = 0; j<480; ++j){
            pix.r = pixel[j][2]; pix.g = pixel[j][1]; pix.b = pixel[j][0];
            pix2 = rgbtohsv(pix);

            if((pix2.h>=ymin)&&(pix2.h<=ymax) && (pix2.s>=umin)&&(pix2.s<=umax) && (pix2.v>=vmin)&&(pix2.v<=vmax))
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

// Returns segmented image of buffer, based on current LUT configuration
void ImageCalibrator::getSegmentedImage(Mat *buffer)
{
    int nRows = 480; int nCols = 480;
    if (buffer->isContinuous()){ nCols *= nRows; nRows = 1;}
    Vec3b* pixel,*mPixel;
    long int index = 0;

    for(int i = 0; i < nRows; ++i){
        pixel = buffer->ptr<Vec3b>(i);
        for (int j = 0; j<nCols; ++j){
             index = (pixel[j][2]<<16) + (pixel[j][1]<<8) + (pixel[j][0]);
             if ( YUVLookUpTable[index] == UAV_GREEN_BIT )//se é campo
             {
                 pixel[j][2] = 0;
                 pixel[j][1] = 255;
                 pixel[j][0] = 0;
             } else if ( YUVLookUpTable[index] == UAV_WHITE_BIT )//se é campo
             {
                 pixel[j][2] = 255;
                 pixel[j][1] = 255;
                 pixel[j][0] = 255;
             }else if ( YUVLookUpTable[index] == UAV_ORANGE_BIT )//se é bola
             {
                 pixel[j][2] = 255;
                 pixel[j][1] = 0;
                 pixel[j][0] = 0;
             }else if ( YUVLookUpTable[index] == UAV_BLACK_BIT)//se é obstaculo
             {
                 pixel[j][2] = 0;
                 pixel[j][1] = 0;
                 pixel[j][0] = 0;
             }else if ( YUVLookUpTable[index] == UAV_NOCOLORS_BIT)//se é mascara ou desconhecido
             {
                 pixel[j][2] = 127;
                 pixel[j][1] = 127;
                 pixel[j][0] = 127;
             }
        }
    }

}

// Updates Look up Table values of the selected pixel, with a radious of rad around it, for the given label
void ImageCalibrator::updateLookUpTable(Mat *buffer, int x, int y, int label, int rad)
{
    for(int i=x-rad;i<=x+rad;i++){
        for(int j=y-rad;j<=y+rad;j++){
            if(i<0 || i>480) return;
            if(j<0 || j>480) return;

            Vec3b *color = buffer->ptr<Vec3b>(j);
            long int index = (color[i][2]<<16) + (color[i][1]<<8) + (color[i][0]);
            YUVLookUpTable[index] = label;
        }
    }
}

// Resets the look up table (puts everything to zero)
void ImageCalibrator::resetLookUpTable()
{
    memset(&YUVLookUpTable,UAV_NOCOLORS_BIT,LUT_SIZE);
}

// Generates new look up table given the ranges in values
void ImageCalibrator::generateLookUpTable(int values[4][3][2])
{
    int y,u,v;
    unsigned int index;
    for (int r=0; r<256; r++) // classify every RGB color into our LUT
        for (int g=0; g<256; g++)
            for (int b=0; b<256; b++)
            {
                 y=(9798*r+19235*g+3736*b)>>15;u=((18514*(b-y))>>15)+128;v=((23364*(r-y))>>15)+128;
                 index = (r<<16)+(g<<8)+b;

                //-- initialize on update --
                YUVLookUpTable[index] = UAV_NOCOLORS_BIT;
                //-- Reference Colour range --
                if (((y>=values[1][0][0]) && (y<=values[1][0][1])) && ((u>=values[1][1][0]) && (u<=values[1][1][1])) &&
                        ((v>=values[1][2][0]) && (v<=values[1][2][1]))){
                    YUVLookUpTable[index] = UAV_WHITE_BIT;
                }else if (((y>=values[0][0][0]) && (y<=values[0][0][1])) && ((u>=values[0][1][0]) && (u<=values[0][1][1])) &&
                          ((v>=values[0][2][0]) && (v<=values[0][2][1]))){
                    YUVLookUpTable[index] = UAV_GREEN_BIT;
                } else if (((y>=values[2][0][0]) && (y<=values[2][0][1])) && ((u>=values[2][1][0]) && (u<=values[2][1][1])) &&
                           ((v>=values[2][2][0]) && (v<=values[2][2][1]))){
                    YUVLookUpTable[index] = UAV_BLACK_BIT;
                }else if (((y>=values[3][0][0]) && (y<=values[3][0][1])) && ((u>=values[3][1][0]) && (u<=values[3][1][1])) &&
                          ((v>=values[3][2][0]) && (v<=values[3][2][1]))){
                    YUVLookUpTable[index] = UAV_ORANGE_BIT;
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
