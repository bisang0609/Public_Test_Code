#include "osdwidget.h"

#include <QPainter>
#include "common.h"


#include <QtDebug>


/* Spot 등 환부 영상 위에 그려지는 Over Screen Display */

OSDWidget::OSDWidget(QWidget* parent) :
    QWidget(parent)
{
    setAttribute(Qt::WA_TranslucentBackground);
}


OSDWidget::~OSDWidget()
{

}

void OSDWidget::setCuRAS(AgentCuRAS* agentCuRAS)
{
    this->agentCuRAS = agentCuRAS;
}


void OSDWidget::clear()
{
    mode = CLEAR;
}


int OSDWidget::getScaled(int _value)
{
    static bool _once = false;

    if (this->agentCuRAS == nullptr && !_once)
    {
        qInfo() << "agentCuRAS not assigned...(OSDWidget)";
        _once = true;
    }

    return (double)(_value*IMG_DISPLAY_WIDTH)/IMAGE_WIDTH;
}

void OSDWidget::drawCross(cv::Point _point)
{
    mode = CROSS;
    point = _point;
}


void OSDWidget::drawCross(QPainter& painter)
{
    painter.setPen(QPen(Qt::red, 2, Qt::SolidLine, Qt::SquareCap));

#define CROSS_SIZE  40
    painter.drawLine(getScaled(point.x)-CROSS_SIZE, getScaled(point.y),
                     getScaled(point.x)+CROSS_SIZE, getScaled(point.y));
    painter.drawLine(getScaled(point.x), getScaled(point.y)-CROSS_SIZE,
                     getScaled(point.x), getScaled(point.y)+CROSS_SIZE);
}

void OSDWidget::_drawGuide(QPainter& painter, cv::Point _point)
{
    painter.setPen(QPen(Qt::green, 2, Qt::SolidLine, Qt::SquareCap));
    painter.drawEllipse(QPoint(getScaled(_point.x), getScaled(_point.y)),
                        getScaled(agentCuRAS->getSpotRadiusSize()), getScaled(agentCuRAS->getSpotRadiusSize()));
}

#if 1       // 20230222 JYH Append
void OSDWidget::_drawGuide_Color(QPainter& painter, cv::Point _point, QColor color)
{
    painter.setPen(QPen(color, 2, Qt::SolidLine, Qt::SquareCap));
    painter.drawEllipse(QPoint(getScaled(_point.x), getScaled(_point.y)),
                        getScaled(agentCuRAS->getSpotRadiusSize()), getScaled(agentCuRAS->getSpotRadiusSize()));
}
#endif       // 20230222 JYH Append

void OSDWidget::_drawGuide(QPainter& painter, cv::Point _point, Qt::BrushStyle style)
{
    painter.setBrush(style);
    _drawGuide(painter, _point);
}

void OSDWidget::_drawGuide(QPainter& painter, cv::Point _point, const QBrush &brush)
{
    painter.setBrush(Qt::SolidPattern);
    painter.setBrush(brush);

    _drawGuide(painter, _point);
}

void OSDWidget::drawSingleGuide(Point _point)
{
    mode = SINGLE_GUIDE;
    point = _point;
}

void OSDWidget::drawSingleGuide(QPainter& painter, cv::Point _point)
{
    _drawGuide(painter, _point, Qt::NoBrush);
}

void OSDWidget::drawMultipleGuide(std::vector<cv::Point> _points)
{
    mode = MULTIPLE_GUIDE;
    points = _points;
    targetSize = 0;
}

void OSDWidget::drawMultipleGuideEx(std::vector<cv::Point> _points, int _targetSize)
{
    drawMultipleGuide(_points);
    targetSize = _targetSize;
}

#define RECT_LEFT		5
#define RECT_TOP		5
#define RECT_WIDTH		230
#define RECT_HEIGHT		230

void OSDWidget::drawDetectedGuide(QPainter& painter, std::vector<cv::Point>& _points)
{
    if (_points.size() > 0)
    {
        painter.setPen(QPen(Qt::green, 2, Qt::SolidLine, Qt::SquareCap));
        painter.drawRect(getScaled(RECT_LEFT+5), getScaled(RECT_TOP+5), getScaled(RECT_WIDTH-10), getScaled(RECT_HEIGHT-10));
    }
}

void OSDWidget::drawMultipleGuide(QPainter& painter, std::vector<cv::Point>& _points)
{
    drawDetectedGuide(painter, _points);

    /* Multiple Shot의 경우, 타겟영역을 디스플레이 */
    if (targetSize)
    {
        painter.setPen(QPen(Qt::red, 2, Qt::SolidLine, Qt::SquareCap));
        painter.drawEllipse(QPoint(getScaled(CENTER_X), getScaled(CENTER_Y)),
                            getScaled(targetSize), getScaled(targetSize));

    }

    for(vector<Point>::iterator _point = _points.begin(); _point != _points.end(); _point++)
    {
        drawSingleGuide(painter, *_point);
    }
}


void OSDWidget::drawShotOk()
{
    mode = SHOT_OK;
    update();
}

void OSDWidget::drawShotOk(uint8_t _value)
{
    mode = MULTIPLE_SHOT_OK;
    indexOfShot = _value;
    update();
}

void OSDWidget::drawShotOk(QPainter& painter, cv::Point _point)
{
#if 1       // 20230222 JYH Append
    //_drawGuide_Color(painter, _point, QColor(32, 119, 68, 255));
    _drawGuide_Color(painter, _point, QColor(36, 134, 77, 255));
#else       // 20230222 JYH Append
    _drawGuide(painter, _point, Qt::green);
#endif       // 20230222 JYH Append
}

void OSDWidget::drawShotOk(QPainter& painter, std::vector<cv::Point>& _points, uint8_t _index)
{
    int _i = 0;

    drawDetectedGuide(painter, _points);

    for(vector<Point>::iterator _point = _points.begin(); _point != _points.end(); _point++)
    {
        drawSingleGuide(painter, *_point);

        if (_i < _index)
#if 1       // 20230222 JYH Append
        {
            QColor green05 = Qt::blue;
            green05.setAlphaF( 0.05 );
            //painter.setBrush( green05 );
            //_drawGuide(painter, _points[_i++]);
            //_drawGuide_Color(painter, _points[_i++], QColor(32, 119, 68, 255));
            _drawGuide_Color(painter, _points[_i++], QColor(36, 134, 77, 255));
        }
#else       // 20230222 JYH Append
            _drawGuide(painter, _points[_i++], Qt::green);
#endif       // 20230222 JYH Append
    }
}


void OSDWidget::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    QPainter painter(this);

    switch(mode)
    {
#if 0
    case CLEAR:
        //painter.setCompositionMode(QPainter::CompositionMode_Overlay);
        //painter.eraseRect(event->rect());
        break;
#endif
    case CROSS:
        drawCross(painter);
        break;
    case SINGLE_GUIDE:
        drawSingleGuide(painter, this->point);
        break;
    case MULTIPLE_GUIDE:
        drawMultipleGuide(painter, this->points);
        break;
    case SHOT_OK:
        drawShotOk(painter, this->point);
        break;
    case MULTIPLE_SHOT_OK:
        drawShotOk(painter, this->points, indexOfShot);
        break;
    }
}
