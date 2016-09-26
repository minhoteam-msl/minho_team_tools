#include "mainwindow.h"
#include "ui_mainwindow.h"

MainWindow::MainWindow(int robot_id, bool real_robot, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
   ui->setupUi(this);
}

MainWindow::~MainWindow()
{  
    delete ui;
}
