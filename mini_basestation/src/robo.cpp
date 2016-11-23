#include "robo.h"
#include <QDebug>
#include <QTime>

Robo::Robo(MainWindow *mainw, int number)
{
    myNumber = number;
    diameter = mainw->getRoboTSize();
    factor = mainw->getFactor();
    AngleDirection1 = 0;
    pressed = false;
    setFlag(ItemIsSelectable);
    setFlag(ItemIsMovable);
    setFlag(ItemSendsScenePositionChanges);
    width= diameter/factor;
    height=width;
    myMain = mainw;
    Pen.setColor(QColor(Qt::black));
    Pen.setWidth(3);
    font.setFamily("Arial");
    font.setPointSize(6);
    pixmap3.load(":/robot.png");
    alfa = 90;
    r = 0;
    g = 0;
    b = 0;
    has_ball = false;

}

QRectF Robo::boundingRect() const
{
    return QRectF(0,0,width,height);
}

void Robo::paint(QPainter *painter, const QStyleOptionGraphicsItem *option, QWidget *widget)
{
    Q_UNUSED(option);
    Q_UNUSED(widget);

    QPainterPath ellipsePath;
    ellipsePath.moveTo(width/2,height/2);
    ellipsePath.arcTo(boundingRect(), AngleDirection1+30, 300);

    QBrush brush(QColor(r,g,b));

    if(pressed)
    {
        brush.setColor(Qt::gray);
    }
    else
    {
        brush.setColor(QColor(r,g,b));
    }

    painter->setBrush(brush);
    painter->drawPath(ellipsePath);

    if(has_ball){
        QPainterPath ballCircle;
        ballCircle.moveTo(width/2,height/2);
        ballCircle.arcTo(boundingRect(), AngleDirection1+30, 300);
        Pen.setColor(QColor(255,0,255));
        painter->setPen(Pen);
        painter->drawPath(ballCircle);
        Pen.setColor(QColor(0,0,0));
    }

    if(numberSet!=""){
        Pen.setColor(QColor(255,255,255));
        painter->setPen(Pen);
        painter->setFont(font);
        int px = width/2;
        int py = height/2;
        float dx = width/8;
        float dy = height/8;
        float dangle = (float)getAngle()*(M_PI/180.0);
        px -= cos(dangle)*dx-sin(dangle)*dy;
        py -= sin(dangle)*dx+cos(dangle)*dy;
        painter->drawText(px,py,numberSet);
    }

}

void Robo::setNumber(QString number)
{
    numberSet = number;
}

void Robo::setclicked()
{
    pressed = true;
}

void Robo::setNotclicked()
{
    pressed = false;
}

void Robo::setAngle(float angle)
{
    angleRaw = angle;
    angle = -angle;
    if(angle<alfa)angle = angle + 360 - alfa;
    else angle = angle -alfa;
    AngleDirection1 = angle;
    update();
}

int Robo::getAngle()
{
    return angleRaw;
}

void Robo::setColor(int red, int blue, int green)
{
    r = red;
    g = blue;
    b = green;
}

void Robo::setPosition(int x, int y)
{
    this->setPos(x-width/2,y-height/2);
}

void Robo::setBallCatch(bool state)
{
    has_ball = state;
}

void Robo::mousePressEvent(QGraphicsSceneMouseEvent *event)
{
    pressed = true;
    update();
    QGraphicsItem::mousePressEvent(event);
}


void Robo::mouseReleaseEvent(QGraphicsSceneMouseEvent *event)
{
    myMain->setRobotLablePosition(this->pos().x()+width/2,this->pos().y()+height/2,this->getAngle(),this->myNumber,false);
    pressed = false;
    update();
    QGraphicsItem::mouseReleaseEvent(event);
}

void Robo::mouseMoveEvent(QGraphicsSceneMouseEvent *event)
{
    //if(myMain->OnMove)
    QGraphicsItem::mouseMoveEvent(event);
}

QVariant Robo::itemChange(GraphicsItemChange change, const QVariant &value)
{
    if (change == ItemPositionChange && scene()) {
            // value is the new position.
            QPointF newPos = value.toPointF();
            QRectF rect = scene()->sceneRect();
            rect.setX(rect.x()-(width/2));
            rect.setWidth(rect.width()-(width/2));
            rect.setY(rect.y()-(height/2)+2);
            rect.setHeight(rect.height()-(height/2)-2);

            if (!rect.contains(newPos)) {
                // Keep the item inside the scene rect.
                newPos.setX(qMin(rect.right()-(diameter/2)/factor, qMax(newPos.x(), rect.left()+(diameter/2)/factor)));
                newPos.setY(qMin(rect.bottom()-(diameter/2)/factor, qMax(newPos.y(), rect.top()+(diameter/2)/factor)));
                return newPos;
            }
        }
    return QGraphicsItem::itemChange(change, value);
}

void Robo::wheelEvent(QGraphicsSceneWheelEvent *event)
{
    if(event->delta() > 0)
        setAngle(this->getAngle()-5);
    else
        setAngle(this->getAngle()+5);

    if(this->getAngle() < 0)
    setAngle(this->getAngle()+360);
    else if(this->getAngle() > 360)
    setAngle(this->getAngle()-360);

    myMain->setRobotLablePosition(this->pos().x()+width/2,this->pos().y()+height/2,this->getAngle(),this->myNumber,false);
    update();
}
