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

enum UAV_SCANLINES {UAV_HORIZONTAL = 0, UAV_VERTICAL, UAV_RADIAL, UAV_CIRCULAR};
enum UAV_COLORS {UAV_BLUE=0, UAV_YELLOW, UAV_ORANGE, UAV_GREEN, UAV_WHITE, UAV_BLACK, UAV_CYAN, UAV_MAGENTA, UAV_NOCOLORS };
enum UAV_COLORS_BIT {UAV_ORANGE_BIT = 32, UAV_BLACK_BIT = 4, UAV_GREEN_BIT = 16,
    UAV_WHITE_BIT = 8, UAV_BLUE_BIT = 128, UAV_YELLOW_BIT = 64, UAV_CYAN_BIT = 2,
    UAV_MAGENTA_BIT = 1, UAV_NOCOLORS_BIT = 0};

typedef enum LABEL_t {FIELD = 0, LINE, BALL, OBSTACLE} LABEL_t;
typedef enum COMPONENT_t { H = 0, S, V} COMPONENT_t;
typedef enum RANGE_t {MIN = 0, MAX} RANGE_t;

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
    int current_configuration[4][3][2];
    void updateCurrentConfiguration(LABEL_t label, COMPONENT_t component, RANGE_t range, int value);
    /* Vision Variables */
    Mat element;
    Mat processed,buffer;
};

#endif // IMAGECALIBRATOR_H
