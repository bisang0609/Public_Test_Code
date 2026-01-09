#ifndef OSDWIDGET_H
#define OSDWIDGET_H

#include <QObject>
#include <QWidget>
#include <QPaintEvent>

#include <opencv2/imgproc.hpp>

#include "agent_curas.h"

using namespace std;
using namespace cv;

enum {
    CLEAR,
    CROSS = 1,
    SINGLE_GUIDE,
    MULTIPLE_GUIDE,
    SHOT_OK,
    MULTIPLE_SHOT_OK,
};

class OSDWidget : public QWidget
{
    Q_OBJECT

public:
    explicit OSDWidget(QWidget *parent = 0);
    ~OSDWidget();

    void setCuRAS(AgentCuRAS* agentCuRAS);
    void clear();

protected:
    void paintEvent(QPaintEvent *event);

private:
    AgentCuRAS* agentCuRAS;

    uint8_t     mode;
    uint8_t     indexOfShot;    /* MultiShot의 인텍스 */

    cv::Point       point;
    std::vector<cv::Point> points;

    /* Multiple Shot의 경우, 타켓을 디스플레이 */
    uint8_t         targetSize;

    int getScaled(int _value);

    void drawCross(QPainter &painter);
    void _drawGuide(QPainter& painter, cv::Point _point);
    void _drawGuide(QPainter& painter, cv::Point _point, Qt::BrushStyle style);
    void _drawGuide(QPainter& painter, cv::Point _point, const QBrush &brush);
#if 1       // 20230222 JYH Append
    void _drawGuide_Color(QPainter& painter, cv::Point _point, QColor color);

    //void _drawGuide_Blue(QPainter& painter, cv::Point _point);
#endif       // 20230222 JYH Append
    void drawSingleGuide(QPainter& painter, cv::Point _point);
    void drawMultipleGuide(QPainter& painter, std::vector<cv::Point>& _points);
    void drawShotOk(QPainter& painter, cv::Point _point);
    void drawShotOk(QPainter& painter, std::vector<cv::Point>& _points, uint8_t _index);
    void drawDetectedGuide(QPainter& painter, std::vector<cv::Point>& _points);

public slots:
    void drawCross(cv::Point _point);
    void drawSingleGuide(cv::Point _point);
    void drawMultipleGuide(std::vector<cv::Point> _points);
    void drawMultipleGuideEx(std::vector<cv::Point> _points, int _targetSize);
    void drawShotOk();
    void drawShotOk(uint8_t _value);
};

#endif // OSDWIDGET_H
