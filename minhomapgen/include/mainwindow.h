#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QFileDialog>
#include <QCloseEvent>
#include <QDebug>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <QTimer>
#include "types.h"

using namespace cv;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_bt_gen_clicked();
    void loadFieldConfiguration(QString file_);
    void saveMapConfiguration(QString file_);
    void drawOnPoint(Point pt);
    void drawField();
    void display();
    void on_bt_load_clicked();
    Point3i distanceToTarget(double angle,Point pt);
    Point3i getNearestDistance(Point pt);
    bool isTarget(Point pt);
    void on_sp_output_valueChanged(int arg1);
    void on_sp_scale_valueChanged(int arg1);
    void destroyCvWindows();
    void on_rb_fullfield_clicked();
    void closeEvent(QCloseEvent *event);
private:
    Ui::MainWindow *ui;
    fieldDimensions fieldAnatomy;
    Mat field, model;
    QTimer *displayTimer;
    Point mouse;
    int scale, outputResolution;
    bool fieldloaded;
};

#endif // MAINWINDOW_H
