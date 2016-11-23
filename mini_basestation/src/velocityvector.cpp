#include "velocityvector.h"


VelocityVector::VelocityVector(MainWindow *mainw)
{
    myMain = mainw;
    red = 255;
    green = 0;
    blue = 0;

    x1 = 0;
    y1 = 0;
    x2 = 0;
    y2 = 0;
}

QRectF VelocityVector::boundingRect() const
{
    return QRectF(0,0,0,0);
}

void VelocityVector::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);


    QBrush brush(Qt::gray);
    brush.setColor(QColor(red,green,blue));
    painter->setPen(QPen(QColor(red,green,blue)));

    painter->setBrush(brush);

    line.setLine(x1,y1,x2,y2);

    painter->drawLine(line);

}


void VelocityVector::setColor(int r, int g, int b)
{
    red = r;
    green = g;
    blue = b;
}

void VelocityVector::setPosition(int x, int y, int x_2, int y_2)
{
    x1 = x;
    x2 = x_2;
    y1 = y;
    y2 = y_2;
}


