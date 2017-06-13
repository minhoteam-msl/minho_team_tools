#include "mainwindow.h"
#include "ui_mainwindow.h"

static void onMouse(int event, int x, int y, int flags, void* userdata)
{
    Q_UNUSED(flags);
    if(event==EVENT_MOUSEMOVE){
        Point *mouse = (Point *)userdata;
        mouse->x = x;
        mouse->y = y;
    }
}

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    scale = ui->sp_scale->value(); // 1px -> scale mm
    outputResolution = ui->sp_output->value();//in mm
    displayTimer = new QTimer();
    fieldloaded = false;
    connect(displayTimer,SIGNAL(timeout()),this,SLOT(display()));
    mouse = Point(-1,-1);
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_bt_gen_clicked() //Generate and Save Map File
{
    QString path = QString(getenv("HOME"))+"/Common/Fields";
    QString filename = QFileDialog::getSaveFileName(this,
    tr("Save map File"), path, tr("Map Files (*.map)"));

    if(filename!=""){
        if(!filename.contains(".map")){
            if(filename.contains(".")){
                //Correct
                int idx = filename.indexOf(".");
                filename = filename.left(idx-1);
                filename+=".map";
            } else {
                filename += ".map";
            }
        }
        //Generate and save map file
        saveMapConfiguration(filename);
    }
}

void MainWindow::loadFieldConfiguration(QString file_)
{
    // Create view of current Field
    QFile file(file_);
    if(!file.open(QIODevice::ReadOnly)) {
        qDebug() <<"✖ Error reading " << file_;
        return;
    } QTextStream in(&file);

    QString value; int counter = 0;
    while(!in.atEnd()){
       value = in.readLine();
       fieldAnatomy.dimensions[counter] = value.right(value.size()-value.indexOf('=')-1).toInt();
       counter++;
    }

    fieldloaded = true;
    file.close();
    drawField();
}

void MainWindow::saveMapConfiguration(QString file_)
{
    displayTimer->stop();
    QFile file(file_);
    if(!file.open(QIODevice::WriteOnly) || !fieldloaded) {
        qDebug() <<"✖ Error trying to write to " << file_;
        return;
    } QTextStream out(&file);

    int counter = 0;
    float divider = 1.0; // default to mm output

    int step = outputResolution/scale; //step in pixels
    ui->pb_write->setValue(counter);
    ui->pb_write->setMaximum((((field.rows)/step)+1)*(((field.cols+1)/step)+1));
    QString uns;
    QString str = "";
    if(ui->cb_units->currentIndex()==1){
        divider = 1000.0; // if set to m output
        uns="m";
    } else { // mm
        uns="mm";
    }

    str += QString::number((float)(fieldAnatomy.fieldDims.TOTAL_LENGTH*scale)/1000.0);
    str+=",";
    str += QString::number((float)(fieldAnatomy.fieldDims.TOTAL_WIDTH*scale)/1000.0);
    str+=",";
    str+=QString::number((((field.rows)/step)+1)*(((field.cols+1)/step)+1))+",Field,"+uns+"\n";
    out << str;
    QString output = "";
    double xReal = 0.0, yReal = 0.0;
    double compX = 0.0, compY = 0.0;

    if(!ui->rb_fullfield->isChecked()){
        compX = ((fieldAnatomy.fieldDims.TOTAL_LENGTH/2)*scale);
        compY = ((fieldAnatomy.fieldDims.TOTAL_WIDTH/2)*scale);
    }

    if(!ui->rb_fullfield->isChecked()){
        for(int x=0;x<=field.cols;x+=step){
            xReal = ((double)x*(double)scale-(double)compX)/divider; //to meters/millimeters
            for(int y=0;y<=field.rows;y+=step){
                Point pt = Point(x,y);
                Point3i ret = getNearestDistance(pt);
                yReal = ((double)y*(double)scale-(double)compY)/divider; //to meters/millimeters
                if(ui->cb_units->currentIndex()==1){ //meters
                    output = "("+QString::number(xReal,'f',2)+","+QString::number(yReal,'f',2)+"):"
                            +QString::number(ret.z/divider,'f',3)+"\n";
                } else {
                    output = "("+QString::number((int)xReal)+","+QString::number((int)yReal)+"):"
                            +QString::number((int)ret.z)+"\n";
                }
                out << output;
                waitKey(1);
                counter++;
                ui->pb_write->setValue(counter);
            }
        }
    } else {
    
        for(int y=0;y<=field.rows;y+=step){
            yReal = ((double)y*(double)scale-(double)compY)/divider; //to meters/millimeters
            for(int x=0;x<=field.cols;x+=step){
                Point pt = Point(x,y);
                Point3i ret = getNearestDistance(pt);
                xReal = ((double)x*(double)scale-(double)compX)/divider; //to meters/millimeters
                if(ui->cb_units->currentIndex()==1){ //meters
                    output = "("+QString::number(xReal,'f',2)+","+QString::number(yReal,'f',2)+"):"
                            +QString::number(ret.z/divider,'f',3)+"\n";
                } else {
                    output = "("+QString::number((int)xReal)+","+QString::number((int)yReal)+"):"
                            +QString::number((int)ret.z)+"\n";
                }
                out << output;
                waitKey(1);
                counter++;
                ui->pb_write->setValue(counter);
            }
        }
    }

    waitKey(1000);
    ui->pb_write->setValue(0);
    file.close();
    displayTimer->start(30);
}

void MainWindow::drawOnPoint(Point pt)
{
    field.copyTo(model);
    Point3i ret = getNearestDistance(pt);
    line(model,pt,Point(ret.x,ret.y),Scalar(0,0,255),1);
    QString dist = QString::number(ret.z)+"mm";
    putText(model,dist.toStdString(),Point((field.cols/2)*0.8,0.05*field.rows),CV_FONT_HERSHEY_COMPLEX,0.5,Scalar(255,0,255));
    imshow("Map",model);
    int userret = waitKey(5);

    if(userret==27){ // Pressed ESC
        displayTimer->stop();
        QTimer::singleShot(100, this, SLOT(destroyCvWindows()));
    }
}

void MainWindow::drawField()
{
    if(ui->rb_fullfield->isChecked()){ // Generate only area and goalie for LIDAR map
        // This map is checked and correct !
        int edge1 = fieldAnatomy.fieldDims.AREA_WIDTH2/scale;
        int edge2 = fieldAnatomy.fieldDims.AREA_LENGTH2/scale;
        int edge3 = (fieldAnatomy.fieldDims.AREA_LENGTH2/scale)
                + (fieldAnatomy.fieldDims.TOTAL_LENGTH-fieldAnatomy.fieldDims.LENGTH)/(2*scale);
        int line = fieldAnatomy.fieldDims.LINE_WIDTH/scale;
        int midY = edge1/2;
        field = Mat(edge1,edge3,CV_8UC3,Scalar(0,120,0));
        //Bigger Area
        rectangle(field,Rect(Point(0+line/2,0+line/2),Point(edge2-line/2,edge1-line/2)),Scalar(255,255,255),line);
        //Smaller Area
        rectangle(field,Rect(Point(edge2-fieldAnatomy.fieldDims.AREA_LENGTH1/scale+line/2,
                        midY-(fieldAnatomy.fieldDims.AREA_WIDTH1/(2*scale))+line/2),
                             Point(edge2-line/2,
                        midY+(fieldAnatomy.fieldDims.AREA_WIDTH1/(2*scale))-line/2)),Scalar(255,255,255),line);
        //Posts
        int centerxp = edge2-line-1;
        int centeryp1 = midY-fieldAnatomy.fieldDims.GOALIE_LENGTH/(2*scale)-fieldAnatomy.fieldDims.GOALIE_POST_WIDTH/(scale);
        rectangle(field,Rect(Point(centerxp,centeryp1),Size(fieldAnatomy.fieldDims.GOALIE_POST_WIDTH/(scale)
                                                      ,fieldAnatomy.fieldDims.GOALIE_POST_WIDTH/(scale))),Scalar(255,0,0),-1);
        int centeryp2 = midY+fieldAnatomy.fieldDims.GOALIE_LENGTH/(2*scale);
        rectangle(field,Rect(Point(centerxp,centeryp2),Size(fieldAnatomy.fieldDims.GOALIE_POST_WIDTH/(scale)
                                                      ,fieldAnatomy.fieldDims.GOALIE_POST_WIDTH/(scale))),Scalar(255,0,0),-1);
        //sideplates
        int ht = fieldAnatomy.fieldDims.GOALIE_POST_WIDTH/(2*scale)-fieldAnatomy.fieldDims.GOALIE_BOARD_WIDTH/(2*scale);
        int t = fieldAnatomy.fieldDims.GOALIE_BOARD_WIDTH/(scale);
        int centerxside = centerxp+fieldAnatomy.fieldDims.GOALIE_POST_WIDTH/scale;
        rectangle(field,Rect(Point(centerxside,centeryp1+ht),
                             Size(fieldAnatomy.fieldDims.GOALIE_WIDTH/scale,t)),Scalar(255,0,0),-1);
        rectangle(field,Rect(Point(centerxside,centeryp2+ht),
                             Size(fieldAnatomy.fieldDims.GOALIE_WIDTH/scale,t)),Scalar(255,0,0),-1);
        //backplate
        int bckplate_len = 2*t+(fieldAnatomy.fieldDims.GOALIE_LENGTH+
                               (fieldAnatomy.fieldDims.GOALIE_POST_WIDTH-fieldAnatomy.fieldDims.GOALIE_BOARD_WIDTH))/scale;
        rectangle(field,Rect(Point(centerxside+fieldAnatomy.fieldDims.GOALIE_WIDTH/scale,centeryp1+ht),
                             Size(t,bckplate_len)),Scalar(255,0,0),-1);
        cv::line(field,Point(0,midY),Point(edge3,midY),Scalar(255,255,255));
    }else {
        //using 1px -> FACTORcm relation
        int anchorPoint1 = 0,anchorPoint2 = 0;
        for(int i=0;i<23;i++) fieldAnatomy.dimensions[i] /= scale;
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
        rectangle(field,Point(fieldAnatomy.fieldDims.TOTAL_LENGTH/2-500/scale,anchorPoint2+fieldAnatomy.fieldDims.WIDTH+50/scale),
                 Point(fieldAnatomy.fieldDims.TOTAL_LENGTH/2+500/scale,anchorPoint2+fieldAnatomy.fieldDims.WIDTH+250/scale),
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

    field.copyTo(model);
    displayTimer->start(30);
}

void MainWindow::display()
{
    // Draw nearest line to goalie
    if(!displayTimer->isActive()){ destroyAllWindows(); return; }

    drawOnPoint(mouse);
    setMouseCallback("Map",onMouse,&mouse);
}

void MainWindow::on_bt_load_clicked() //Load field
{
    QString path = QString(getenv("HOME"))+"/Common/Fields";
    QString filename = QFileDialog::getOpenFileName(this,
    tr("Load field File"), path, tr("Field Files (*.view)"));

    if(filename!=""){
        loadFieldConfiguration(filename);
        ui->cb_units->setEnabled(false);
        ui->sp_output->setEnabled(false);
        ui->sp_scale->setEnabled(false);
        ui->rb_fullfield->setEnabled(false);
    }
}

Point3i MainWindow::distanceToTarget(double angle,Point pt)
{
    //InitialPoint : mouse
    // return : targetx,targety,distance(mm)
    int x = pt.x; int y = pt.y; Point3i ret = Point3i(pt.x,pt.y,-1);

    // Forward search
    double dist = 0;
    while(x>=0&&x<=field.cols&&y>=0&&y<=field.rows){
        x = pt.x+cos(angle)*dist; y = pt.y+sin(angle)*dist; dist+=0.5;
        if(isTarget(Point(x,y))){
            ret.x = x; ret.y = y; ret.z = scale*sqrt((x-pt.x)*(x-pt.x)+(y-pt.y)*(y-pt.y));
            break;
        }
    }
    return ret;
}

Point3i MainWindow::getNearestDistance(Point pt)
{
    if(isTarget(pt)){
        return Point3i(pt.x,pt.y,0);
    }

    int nearestDistance = 1000000;
    Point3i nearestPoint;
    for(double angle=0.0;angle<=(2*M_PI);angle+=(M_PI/180.0)){
        Point3i distance = distanceToTarget(angle,pt);
        if(distance.z>=0 && distance.z<nearestDistance){
            nearestDistance = distance.z;
            nearestPoint = distance;
        }
    }
    return nearestPoint;
}

bool MainWindow::isTarget(Point pt)
{
    if(pt.x<0 || pt.x>=field.cols || pt.y<0 || pt.y>=field.rows) return false;
    Vec3b *color = field.ptr<Vec3b>(pt.y);
    if(!ui->rb_fullfield->isChecked()) { if(color[pt.x][0]==255&&color[pt.x][1]==255&&color[pt.x][2]==255) return true; } // white lines
    else { if(color[pt.x][0]==255&&color[pt.x][1]==0) return true; }
    return false;
}

void MainWindow::on_sp_output_valueChanged(int arg1)
{
    outputResolution = arg1;
}

void MainWindow::on_sp_scale_valueChanged(int arg1)
{
    scale = arg1;
}

void MainWindow::destroyCvWindows()
{
    destroyAllWindows();
    ui->cb_units->setEnabled(true);
    ui->sp_output->setEnabled(true);
    ui->sp_scale->setEnabled(true);
    ui->rb_fullfield->setEnabled(true);
}

void MainWindow::on_rb_fullfield_clicked()
{
    if(ui->rb_fullfield->isChecked())ui->sp_output->setValue(50); // default value for LIDAR map
    else ui->sp_output->setValue(100); // default value for full map
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    displayTimer->stop();
    QTimer::singleShot(100, this, SLOT(destroyCvWindows()));
    event->accept();
}
