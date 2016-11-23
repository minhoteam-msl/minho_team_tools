#include "mainwindow.h"
#include "ui_minibs.h"
#include "customscene.h"
#include "desenharcampo.h"
#include "robo.h"
#include "ball.h"
#include "math.h"
#include "velocityvector.h"

#define SGN(x) ((x)>=0?1:-1)

#define official
//#define lar

#define OFFLINE

const int numberRobots = 5;

Ball *ball[numberRobots];
Robo *robo[numberRobots*2];
Ball *mainBall;

VelocityVector *velrb[numberRobots*2];
VelocityVector *velball[numberRobots];

bool limitUp = false;
int roboPass = 1;

struct nodo
{
    float x[numberRobots];
    float y[numberRobots];
    float angle[numberRobots];
    char name[100];
};


MainWindow::MainWindow(bool is_official_field, pthread_t *recv_thread, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    recv_thread_ptr = recv_thread;
    qRegisterMetaType<minho_team_ros::interAgentInfo>("minho_team_ros::interAgentInfo");

    connect(this,SIGNAL(new_robot_info(minho_team_ros::interAgentInfo)),this,SLOT(change_robot_info(minho_team_ros::interAgentInfo)));

    nTaticsLoad = 0;

    minhoGoals=0; awayGoals=0,robotWithBall = 0;

    check_show_home_robots = true,check_show_away_robots = true,check_show_grid_cells = false
                ,check_show_pass_lines = false,check_show_shot_lines = false,check_show_obstacles = false
                ,check_game_direction = false,check_RTDB = false,check_main_loop=true;

    we_have_ball = false;

    
    for(int i=0;i<5;i++){
        robo_state[i] = false;
        robo_state_old[i] = false;
    }

    int id = QFontDatabase::addApplicationFont(":/resources/display.ttf");
    QString family = QFontDatabase::applicationFontFamilies(id).at(0);
    QFont monospace(family);
    monospace.setPixelSize(80);
    //ui->lb_home_score->setFont(monospace);
    //ui->lb_away_score->setFont(monospace);

    //ui->bt_game_direction->installEventFilter(this);


    setRobotLable(ui->state_robo1, false, "1");
    setRobotLable(ui->state_robo2, false, "2");
    setRobotLable(ui->state_robo3, false, "3");
    setRobotLable(ui->state_robo4, false, "4");
    setRobotLable(ui->state_robo5, false, "5");


    // Default Definitions

    TOTAL_LENGTH=20000;
    TOTAL_WIDTH=14000;
    LENGTH=18000;
    WIDTH=12000;
    GOAL_WIDTH=2000;
    GOAL_LENGTH=500;
    LINE_WIDTH=120;
    CENTER_RADIUS=2000;
    SPOT_CENTER=150;
    SPOTS=100;
    AREA_LENGTH1=750;
    AREA_WIDTH1= GOAL_WIDTH + 1500;
    AREA_LENGTH2=2250;
    AREA_WIDTH2= GOAL_WIDTH + 4500;
    DISTANCE_PENALTY=3000;
    RADIUS_CORNER=750;
    ROBOT_DIAMETER=500;
    BALL_DIAMETER = 220;
    FREE_KICK_SPACE = 3000;
    FACTOR=20;// 1 pixel = 2 cm

    if(is_official_field) readDimension("campo-oficial.txt");
    else readDimension("campo-lar.txt");

    QGraphicsView *fieldView = ui->fieldView;
    fieldView->setFixedSize(TOTAL_LENGTH/FACTOR,TOTAL_WIDTH/FACTOR);
    fieldView->setRenderHint(QPainter::Antialiasing);

    scene = new CustomScene(this);

    scene->setSceneRect((-TOTAL_LENGTH/2)/FACTOR,(-TOTAL_WIDTH/2)/FACTOR,(TOTAL_LENGTH)/FACTOR,(TOTAL_WIDTH)/FACTOR);

    QBrush brush(QColor(0,150,0));
    QPen pen;
    pen.setColor(QColor (255,255,255));
    pen.setWidth(1);

    scene->addRect((-TOTAL_LENGTH/2)/FACTOR,(-TOTAL_WIDTH/2)/FACTOR,(TOTAL_LENGTH)/FACTOR,(TOTAL_WIDTH)/FACTOR,pen,brush);

    fieldView->setScene(scene);

    DesenharCampo *campo = new DesenharCampo();
    campo->desenharLinhas(scene,LENGTH,WIDTH,GOAL_WIDTH,GOAL_LENGTH,LINE_WIDTH,CENTER_RADIUS,SPOT_CENTER,SPOTS,AREA_LENGTH1,AREA_WIDTH1,AREA_LENGTH2,AREA_WIDTH2,DISTANCE_PENALTY,RADIUS_CORNER,FACTOR);

    //Pass Line;
    pen_line.setWidth(2);
    pen_line.setColor(QColor(0,255,0));
    pass_line = new QGraphicsLineItem();
    pass_line->setPen(pen_line);

    //Goal Circle
    pen_line.setColor(QColor(255,0,0));
    shot_circle = new QGraphicsEllipseItem();
    shot_circle->setPen(pen_line);
    static_goal_taget_x = ((LENGTH/2)/FACTOR);
    goal_target_y = -5;
    shot_circle->setRect(static_goal_taget_x,goal_target_y,10,10);

    //Goal Line;
    pen_line.setWidth(2);
    pen_line.setColor(QColor(0,255,0));
    shot_line = new QGraphicsLineItem();
    shot_line->setPen(pen_line);

    for(int i=numberRobots;i<10;i++)
    {
        robo[i] = new Robo(this,i+1);
        robo[i]->setNumber(QString::number(i-4));
        robo[i]->setData(0,i);
        
        if(is_official_field) robo[i]->setPosition(getx(1.5)+((i-4)*(ROBOT_DIAMETER*2/FACTOR)),gety(6.2));
        else robo[i]->setPosition(getx(0.5)+((i-4)*(ROBOT_DIAMETER*1.5/FACTOR)),gety(4));

        robotOldPositionsx[i] = robo[i]->pos().x();
        robotOldPositionsy[i] = robo[i]->pos().y();
        robo[i]->setAngle(0);
        robo[i]->setColor(0,0,100);
        scene->addItem(robo[i]);
    }

    for(int i=0;i<numberRobots;i++)
    {
        robo[i] = new Robo(this,i+1);
        robo[i]->setNumber(QString::number(i+1));
        robo[i]->setData(0,i);
        
        if(is_official_field) robo[i]->setPosition(getx(-7)+(i*(ROBOT_DIAMETER*2/FACTOR)),gety(6.2));
        else robo[i]->setPosition(getx(-1.5)+((i-4)*(ROBOT_DIAMETER*1.5/FACTOR)),gety(4));
        
        robotOldPositionsx[i] = robo[i]->pos().x();
        robotOldPositionsy[i] = robo[i]->pos().y();
        robo[i]->setAngle(0);
        robo[i]->setColor(120,10,15);
        scene->addItem(robo[i]);
    }


    for(int i=0;i<numberRobots;i++)
    {
        ball[i] = new Ball(this,i+1,BALL_DIAMETER);
        ball[i]->setNumber(QString::number(i+1));
        ball[i]->setData(0,i);

        if(is_official_field) ball[i]->setPos(getx(-7)+(i*(ROBOT_DIAMETER*2/FACTOR)),gety(6.2));
        else ball[i]->setPos(getx(-1.5)+((i-4)*(ROBOT_DIAMETER*1.5/FACTOR)),gety(4));
        scene->addItem(ball[i]);
    }

    mainBall = new Ball(this,6,BALL_DIAMETER);
    mainBall->setNumber("M");
    mainBall->setData(0,6);
    mainBall->setColor(255,0,0);
    mainBall->setPos(getx(-7)+(5*(ROBOT_DIAMETER*2/FACTOR)),gety(6.2));
    scene->addItem(mainBall);

    for(int i=0;i<numberRobots*2;i++)
    {
        velrb[i] = new VelocityVector(this);
        scene->addItem(velrb[i]);
    }

    for(int i=0;i<numberRobots;i++)
    {
        velball[i] = new VelocityVector(this);
        scene->addItem(velball[i]);
    }

}

MainWindow::~MainWindow()
{
    delete ui;
}

int MainWindow::getFactor()
{
    return FACTOR;
}

int MainWindow::getRoboTSize()
{
    return ROBOT_DIAMETER;
}

void MainWindow::readDimension(QString FileName)
{
    QFile file(":/data/"+FileName);

    if(!file.open(QIODevice::ReadOnly)) {
        QMessageBox::warning(0, "Error", +"data/"+FileName+" "+file.errorString());
    }
    QTextStream in(&file);
    QString line;
    int lineN = 0;
    while(!in.atEnd()) {
        line = in.readLine();

        switch (lineN) {
        case 0:
            line = line.replace("TOTAL_LENGTH=",0);
            TOTAL_LENGTH = line.toInt();
        break;
        case 1:
            line = line.replace("TOTAL_WIDTH=",0);
            TOTAL_WIDTH = line.toInt();
        break;
        case 2:
            line = line.replace("LENGTH=",0);
            LENGTH = line.toInt();
        break;
        case 3:
            line = line.replace("WIDTH=",0);
            WIDTH = line.toInt();
        break;
        case 4:
            line = line.replace("GOAL_WIDTH=",0);
            GOAL_WIDTH = line.toInt();
        break;
        case 5:
            line = line.replace("GOAL_LENGTH=",0);
            GOAL_LENGTH = line.toInt();
        break;
        case 6:
            line = line.replace("LINE_WIDTH=",0);
            LINE_WIDTH = line.toInt();
        break;
        case 7:
            line = line.replace("CENTER_RADIUS=",0);
            CENTER_RADIUS = line.toInt();
        break;
        case 8:
            line = line.replace("SPOT_CENTER=",0);
            SPOT_CENTER = line.toInt();
        break;
        case 9:
            line = line.replace("SPOTS=",0);
            SPOTS = line.toInt();
        break;
        case 10:
            line = line.replace("AREA_LENGTH1=",0);
            AREA_LENGTH1 = line.toInt();
        break;
        case 11:
            line = line.replace("AREA_WIDTH1=",0);
            AREA_WIDTH1 = line.toInt();
        break;
        case 12:
            line = line.replace("AREA_LENGTH2=",0);
            AREA_LENGTH2 = line.toInt();
        break;
        case 13:
            line = line.replace("AREA_WIDTH2=",0);
            AREA_WIDTH2 = line.toInt();
        break;
        case 14:
            line = line.replace("DISTANCE_PENALTY=",0);
            DISTANCE_PENALTY = line.toInt();
        break;
        case 15:
            line = line.replace("RADIUS_CORNER=",0);
            RADIUS_CORNER = line.toInt();
        break;
        case 16:
            line = line.replace("ROBOT_DIAMETER=",0);
            ROBOT_DIAMETER = line.toInt();
        break;
        case 17:
            //line = line.replace("BALL_DIAMETER=",0);
            //BALL_DIAMETER = line.toInt();
        break;
        case 18:
            line = line.replace("FREE_KICK_SPACE=",0);
            FREE_KICK_SPACE = line.toInt();
            break;
        case 19:
            line = line.replace("FACTOR=",0);
            FACTOR = line.toInt();
        break;
        default:
            break;
        }
        lineN++;
    }

}

int MainWindow::getx(float meterX)
{
    return meterX*1000/FACTOR;
}

int MainWindow::gety(float meterY)
{
    return meterY*1000/FACTOR;
}

float MainWindow::getMeterx(int X)
{
    return (float)X*(float)FACTOR/1000.0;
}

float MainWindow::getMetery(int Y)
{
    return (float)Y*(float)FACTOR/1000.0;
}

void MainWindow::setRobotLable(QLabel *lable, bool state, QString number)
{
    if(state){
        QMovie *movie = new QMovie(":/resources/on_"+number+".gif");
        movie->setScaledSize(QSize(20,20));
        QLabel *processLabel = lable;
        processLabel->setMovie(movie);
        processLabel->setToolTip("Online");
        movie->start();
    }
    else{
        QMovie *movie = new QMovie(":/resources/off_"+number+".gif");
        movie->setScaledSize(QSize(20,20));
        QLabel *processLabel = lable;
        processLabel->setToolTip("Offline");
        processLabel->setMovie(movie);
        movie->start();
    }

}

void MainWindow::change_robot_info(minho_team_ros::interAgentInfo incoming_data)
{
    robo[incoming_data.agent_id-1]->setPosition(getx(incoming_data.agent_info.robot_info.robot_pose.x),getx(incoming_data.agent_info.robot_info.robot_pose.y));
    robo[incoming_data.agent_id-1]->setAngle(incoming_data.agent_info.robot_info.robot_pose.z);
    setRobotLablePosition(incoming_data.agent_info.robot_info.robot_pose.x,
    incoming_data.agent_info.robot_info.robot_pose.y,
    incoming_data.agent_info.robot_info.robot_pose.z
    ,incoming_data.agent_id,true);
  
    // Draw robot velocity vector

    //velrb[incoming_data.agent_id-1]->setPosition(getx(incoming_data.agent_info.robot_info.robot_pose.x),getx(incoming_data.agent_info.robot_info.robot_pose.y),getx(incoming_data.agent_info.robot_info.robot_velocity.x),gety(incoming_data.agent_info.robot_info.robot_velocity.y));


    if(incoming_data.agent_info.robot_info.sees_ball)
    {
        robo[incoming_data.agent_id-1]->setBallCatch(incoming_data.agent_info.robot_info.has_ball);
        ball[incoming_data.agent_id-1]->setPosition(getx(incoming_data.agent_info.robot_info.ball_position.x),gety(incoming_data.agent_info.robot_info.ball_position.y));
        
        // Draw ball velocity vector
        //velball[incoming_data.agent_id-1]->setPosition(getx(incoming_data.agent_info.robot_info.ball_position.x),getx(incoming_data.agent_info.robot_info.ball_position.y),getx(incoming_data.agent_info.robot_info.ball_velocity.x),gety(incoming_data.agent_info.robot_info.ball_velocity.y));
        
    } else {
        ball[incoming_data.agent_id-1]->setPosition(-1000,-1000);  
        robo[incoming_data.agent_id-1]->setBallCatch(false);
    }
}

void MainWindow::onUpdateFromRobot(void *data)
{
    // deserialize message
   interAgentInfo incoming_data;
   deserializeROSMessage<interAgentInfo>((udp_packet *)data,&incoming_data);
   delete((udp_packet *)data);

   int agent_id = incoming_data.agent_id;

   if(agent_id<1 || agent_id>=6);
   else {
       emit new_robot_info(incoming_data);
   }

   return;
}

void MainWindow::setRobotLablePosition(float x, float y, int angle, int robot, bool in_meter)
{
    if(in_meter){
       switch (robot) {
          case 1:
          ui->lb_pos_r1->setText("Pose["+QString::number(x,'f',2)+" , "+QString::number(y,'f',2)+" , "+QString::number(angle)+"º"+"]");
              break;
          case 2:
          ui->lb_pos_r2->setText("Pose["+QString::number(x,'f',2)+" , "+QString::number(y,'f',2)+" , "+QString::number(angle)+"º"+"]");
              break;
          case 3:
          ui->lb_pos_r3->setText("Pose["+QString::number(x,'f',2)+" , "+QString::number(y,'f',2)+" , "+QString::number(angle)+"º"+"]");
              break;
          case 4:
          ui->lb_pos_r4->setText("Pose["+QString::number(x,'f',2)+" , "+QString::number(y,'f',2)+" , "+QString::number(angle)+"º"+"]");
              break;
          case 5:
          ui->lb_pos_r5->setText("Pose["+QString::number(x,'f',2)+" , "+QString::number(y,'f',2)+" , "+QString::number(angle)+"º"+"]");
              break;
          default:
              break;
       }
    } else {
       switch (robot) {
          case 1:
          ui->lb_pos_r1->setText("Pose["+QString::number(getMeterx(x),'f',2)+" , "+QString::number(getMetery(y),'f',2)+" , "+QString::number(angle)+"º"+"]");
              break;
          case 2:
          ui->lb_pos_r2->setText("Pose["+QString::number(getMeterx(x),'f',2)+" , "+QString::number(getMetery(y),'f',2)+" , "+QString::number(angle)+"º"+"]");
              break;
          case 3:
          ui->lb_pos_r3->setText("Pose["+QString::number(getMeterx(x),'f',2)+" , "+QString::number(getMetery(y),'f',2)+" , "+QString::number(angle)+"º"+"]");
              break;
          case 4:
          ui->lb_pos_r4->setText("Pose["+QString::number(getMeterx(x),'f',2)+" , "+QString::number(getMetery(y),'f',2)+" , "+QString::number(angle)+"º"+"]");
              break;
          case 5:
          ui->lb_pos_r5->setText("Pose["+QString::number(getMeterx(x),'f',2)+" , "+QString::number(getMetery(y),'f',2)+" , "+QString::number(angle)+"º"+"]");
              break;
          default:
              break;
       }
    }
}

template<typename Message>
void MainWindow::deserializeROSMessage(udp_packet *packet, Message *msg)
{
    ros::serialization::IStream istream(packet->packet, packet->packet_size);
    ros::serialization::deserialize(istream, *msg);
}
