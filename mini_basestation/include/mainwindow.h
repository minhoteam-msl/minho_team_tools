#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QGraphicsLineItem>
#include <QCloseEvent>
#include <QTimer>
#include <QFontDatabase>
#include <QDebug>
#include <QMovie>
#include <QLabel>
#include <QGraphicsOpacityEffect>
#include <QPen>
#include <QDir>
#include "math.h"
#include <QLabel>
#include <vector>
#include "minho_team_ros/interAgentInfo.h"
#include "ros/ros.h"
#include <pthread.h>
#include <signal.h>

using namespace std;

using minho_team_ros::interAgentInfo;
/// \brief struct to represet a udp packet, containing
/// a serialized ROS message
typedef struct udp_packet{
   uint8_t *packet;
   uint32_t packet_size;
}udp_packet;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(bool is_official_field, pthread_t *recv_thread, QWidget *parent = 0);
    ~MainWindow();

    template<typename Message>
    void deserializeROSMessage(udp_packet *packet, Message *msg);
public slots:

    int getFactor();

    int getRoboTSize();

    void setRobotLablePosition(float x,float y,int angle, int robot, bool in_meter);

    void onUpdateFromRobot(void *data);
    
    inline void closeEvent (QCloseEvent *event) { pthread_kill(*recv_thread_ptr,9); event->accept(); }
private slots:

    void readDimension(QString FileName);

    int getx(float meterX);

    int gety(float meterY);

    float getMeterx(int X);

    float getMetery(int Y);

    void setRobotLable(QLabel *lable, bool state, QString number);

    void change_robot_info(minho_team_ros::interAgentInfo incoming_data);

private:
    Ui::MainWindow *ui;

    QGraphicsScene *scene;

    QTimer *timerPassLine,*timerRTDB,*timerRef,*timerMainLoop;

    QPen pen_line;

    // Global Definitions
    int TOTAL_LENGTH;
    int TOTAL_WIDTH;
    int LENGTH;
    int WIDTH;
    int GOAL_WIDTH;
    int GOAL_LENGTH;
    int LINE_WIDTH;
    int CENTER_RADIUS;
    int SPOT_CENTER;
    int SPOTS;
    int AREA_LENGTH1;
    int AREA_WIDTH1;
    int AREA_LENGTH2;
    int AREA_WIDTH2;
    int DISTANCE_PENALTY;
    int RADIUS_CORNER;
    int ROBOT_DIAMETER;
    int BALL_DIAMETER;
    int FREE_KICK_SPACE;
    int FACTOR;// 1 pixel = 2 cm

    int robotOldPositionsx[10],robotOldPositionsy[10];

    bool check_show_home_robots,check_show_away_robots,check_show_grid_cells
            ,check_show_pass_lines,check_show_shot_lines,check_show_obstacles
            ,check_game_direction,check_RTDB,check_main_loop;

    int goal_target_y,static_goal_taget_x;

    QGraphicsLineItem *pass_line,*shot_line;
    QGraphicsEllipseItem *shot_circle;

    struct nodo *robotNode;

    int nTaticsLoad;

    int minhoGoals,awayGoals;

    bool robo_state[5],robo_state_old[5],we_have_ball;

    int robotWithBall;
    
    pthread_t *recv_thread_ptr;
signals:
    void new_robot_info(minho_team_ros::interAgentInfo incoming_data);
};

#endif // MAINWINDOW_H
