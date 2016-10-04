#ifndef IMAGECALIBRATOR_H
#define IMAGECALIBRATOR_H

#include <QFile>
#include <QString>
#include <QTextStream>
#include <QStringList>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "types.h"
#include <iostream>

using namespace std;
using namespace cv;
class ImageCalibrator
{
public:
    ImageCalibrator();
    void variablesInitialization();
    void getBinary(Mat *in, int ymin, int ymax, int umin, int umax, int vmin, int vmax); // Returns thresholded HSV image
    void getSegmentedImage(Mat *buffer); // Returns buffer's segmented image
    void paintPixel(int x, int y, int classifier, Mat *buf); // Paints a certain pixel in the image
    hsv rgbtohsv(rgb in); // Converts rgb to hsv
    void resetLookUpTable();
    void generateLookUpTable(int values[4][3][2]); // Generates look up table based on values' ranges
    void updateLookUpTable(Mat *buffer, int x, int y, int label, int rad);
    int getClassifier(int x,int y, Mat *buffer); // Returns classifier given a pixel and LUT configuration
    /* Other Variables and Lut Variables*/
    int YUVLookUpTable[256*256*256];
    void updateCurrentConfiguration(LABEL_t label, COMPONENT_t component, RANGE_t range, int value);
    labelConfiguration getLabelConfiguration(LABEL_t label);
    /* Vision Variables */
    Mat element;
    Mat processed,buffer;
    lutConfiguration lutconfig;
};

#endif // IMAGECALIBRATOR_H
