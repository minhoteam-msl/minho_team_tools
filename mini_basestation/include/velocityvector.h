#ifndef VELOCITYVECTOR_H
#define VELOCITYVECTOR_H

#include <QPainter>
#include <QGraphicsItem>
#include <QGraphicsLineItem>
#include <QGraphicsScene>
#include "mainwindow.h"

class VelocityVector : public QGraphicsItem
{
public:
    VelocityVector(MainWindow *mainw);
    QRectF boundingRect() const;
    void paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget);
    void setColor(int r,int g,int b);
    void setPosition(int x, int y, int x_2, int y_2);

private:

    MainWindow *myMain;

    int red,green,blue;

    QLine line;

    int x1,y1,x2,y2;
};

#endif // VELOCITYVECTOR_H
