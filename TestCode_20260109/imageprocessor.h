#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <QObject>
#include <QThread>
#include <QDir>

#include <QWaitCondition>
#include <QMutex>

#include "agent_curas.h"
#include "OtaNervusDetector.h"

/* Raw File을 저장하는 방식
 * 지정하지 않으면 Pixel Raw로 저장
 */
#define  SAVE_JPG

class ImageProcessor : public QThread
{
    Q_OBJECT

public:
    explicit ImageProcessor(AgentCuRAS* agentCuRAS = nullptr);
    ~ImageProcessor() override;

    void initDetector(void);

    uint8_t  frame[IMAGE_SIZE*3];
#if 1   // 20221007 JYH Append
    uint8_t  frame_pos;
    QMutex ImagProc_Mutex;
#endif   // 20221007 JYH Append

#if 1   // 20230322 JYH Append Detect Circle 깜박임 속도 조절
    uint8_t  m_nSkipGuide=0;
#endif   // 20230322 JYH Append Detect Circle 깜박임 속도 조절

private:
    AgentCuRAS* agentCuRAS;

    OtaNervus::Detector detectorLaserGuide;
    OtaNervus::Detector detectorWhite;
    OtaNervus::Detector detectorGV;
    OtaNervus::Detector detectorAUTO;

    Mat  srcImageMat;

    vector<Point> points;

    QDir savePathDir;

    QWaitCondition condition;
    QMutex mutex;

    void drawGuide(vector<Point> _points);
    void doDetect(cv::Mat& _in, cv::Mat& _out);
    void doImageProcessing(cv::Mat& _in, cv::Mat& _out);
    void sendImageProcessingOk(std::vector<cv::Point>);
    std::vector<cv::Point> getMultipleShotPoints(uint8_t _mode, cv::Mat& _in);
    std::vector<cv::Point> getMultiple20MMSquare(uint8_t _overlap);

    bool isValidPostion(cv::Point& _position);
    void doCalibration(cv::Mat& _in);

    void _RGB565ToBGR888(cv::Mat& _in, uint8_t* _data);

#ifdef QT_DEBUG
    QTimer          debugTimer;
    int frameCnt = 0;
    int mis_counter = 0;
#endif

protected:
    void run() override;

private slots:
    void setImage(uint8_t* _buffer, uint16_t _size);
    void saveImage(void);

#ifdef QT_DEBUG
    void timeout(void);
#endif


signals:
    void SIG_setImage(unsigned int _width, unsigned int _height, unsigned char*);
    void drawCross(cv::Point);
    void drawSingleGuide(cv::Point);
    void drawMultipleGuide(std::vector<cv::Point>);
    void drawMultipleGuide(std::vector<cv::Point>, int);
    void imageProcessingOk(std::vector<cv::Point>);
    void sig_binImage(const cv::Mat& bin); // bin 영상 전달용 시그널 추가
    void sig_laserImage(const cv::Mat& img); // [추가] 레이저 가이드(녹색선) 영상 시그널
};

#endif // IMAGEPROCESSOR_H
