#ifndef TYPES
#define TYPES

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <QTimer>
#include <QDebug>
#include <QTime>
#include <QFile>
#include <stdlib.h>

#include "ros/ros.h"
#include "ros/console.h"

#define mainPath "catkin_ws/src/minho_team_tools/visualizer/config/main.cfg"
#define cfgFolderPath "catkin_ws/src/minho_team_tools/visualizer/config/"

using namespace std;
using namespace cv;

struct field// Current field definitions
{
    unsigned int FIELD_POSITIONS;
    double FIELD_WIDTH,FIELD_LENGTH;
    double HALF_FIELD_WIDTH, HALF_FIELD_LENGTH;
    float TERM1, TERM2, TERM3;
    unsigned int MAX_LINE_POINTS;
    QString FIELD_NAME;
};

typedef struct gameField
{
    int TOTAL_LENGTH, TOTAL_WIDTH;
    int LENGTH, WIDTH;
    int GOAL_WIDTH, GOAL_LENGTH;
    int LINE_WIDTH;
    int CENTER_RADIUS;
    int SPOT_CENTER;
    int SPOTS;
    int AREA_LENGTH1, AREA_WIDTH1, AREA_LENGTH2, AREA_WIDTH2;
    int DISTANCE_PENALTY;
    int RADIUS_CORNER;
    int ROBOT_DIAMETER;
    int BALL_DIAMETER;
    int FACTOR;
    int GOALIE_LENGTH,GOALIE_WIDTH;
    int GOALIE_POST_WIDTH,GOALIE_BOARD_WIDTH;
} gameField;

typedef union fieldDimensions{
    struct gameField fieldDims;
    int dimensions[23];
} fieldDimensions;

#endif // TYPES

