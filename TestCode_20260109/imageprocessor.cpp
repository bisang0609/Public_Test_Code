#include "imageprocessor.h"
#include <QDateTime>
#include <QCoreApplication>
#include <opencv2/highgui.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/imgcodecs.hpp"
#include "agent_curas.h"
using namespace std;


ImageProcessor::ImageProcessor(AgentCuRAS* _agentCuRAS)
{
#if 1   // 20221007 JYH Append
    frame_pos = 0;
#endif   // 20221007 JYH Append
    this->agentCuRAS = _agentCuRAS;

    initDetector();

    srcImageMat = Mat(IMAGE_HEIGHT, IMAGE_WIDTH, CV_8UC3);

    qRegisterMetaType<cv::Point>("cv::Point");
    qRegisterMetaType<std::vector<Point>>("std::vector<Point>");

#if 1   // 20221007 JYH Append
    connect(_agentCuRAS,SIGNAL(setImage(uint8_t*, uint16_t)),this,SLOT(setImage(uint8_t*, uint16_t)), Qt::AutoConnection);
#else
    connect(_agentCuRAS,SIGNAL(setImage(uint8_t*, uint16_t)),this,SLOT(setImage(uint8_t*, uint16_t)), Qt::QueuedConnection);
#endif   // 20221007 JYH Append

#ifdef QT_DEBUG
    connect(&debugTimer, SIGNAL(timeout()), this, SLOT(timeout()));
    //debugTimer.setInterval(1000);
    debugTimer.start(5000);
#endif
}

ImageProcessor::~ImageProcessor()
{
}

void ImageProcessor::initDetector(void)
{
    /***
     * 레이저 가이드 디텍터 초기화
     */
    detectorLaserGuide.setSigma(1);
#if 1   //20220622 OJH 에이밍빔 최소값 수정(에이밍빔 못잡는 문제)
    detectorLaserGuide.setMinArea(40);
    detectorLaserGuide.setMaxArea(1500);
#else
    detectorLaserGuide.setMinArea(30);  //20);  //30);
    detectorLaserGuide.setMaxArea(2000);
#endif


    /***
     * White 팬 디텍터 초기화
     */
    detectorWhite.setShape(OtaNervus::Detector::Shape::LINE);
    detectorWhite.setColor(OtaNervus::Detector::Color::WHITE);
    detectorWhite.setSigma(5);
    detectorWhite.setBlock(95);
    detectorWhite.setC(-2);
    detectorWhite.setKernel(5);
    detectorWhite.setMinArea(1000);
    detectorWhite.setMaxArea(40000);

    /***
     * GV 팬 디텍터 초기화
     */
    detectorGV.setShape(OtaNervus::Detector::Shape::DOT);
    detectorGV.setColor(OtaNervus::Detector::Color::BLACK);
    detectorGV.setBlock(21);
    detectorGV.setC(7);
    detectorGV.setKernel(3);
    detectorGV.setDotKernel(15);
    detectorGV.setMinArea(5);
    detectorGV.setMaxArea(300);

    /***
     * AUTO 디텍터 초기화
     */
    detectorAUTO.setShape(OtaNervus::Detector::Shape::AUTO);
    detectorAUTO.setSigma(7);
    detectorAUTO.setKernel(7);
    detectorAUTO.setMinArea(750);
    detectorAUTO.setMaxArea(40000);
    detectorAUTO.setBlock(255);
    detectorAUTO.setC(2);
}


void ImageProcessor::drawGuide(vector<Point> _points)
{
    if (agentCuRAS->isSkinTouched())
    {
        if (agentCuRAS->isFootSwitched())
            return;

        switch (agentCuRAS->getShotMode())
        {
        case SINGLE_NONE:
        case SINGLE_1HZ:
        case SINGLE_2HZ:
            emit drawSingleGuide(_points[0]);
            break;
        case MULTIPLE_6MM:
            emit drawMultipleGuide(_points, MULTIPLE_6MM_TARGET_RADIUS);
            break;
        case MULTIPLE_10MM:
            emit drawMultipleGuide(_points, MULTIPLE_10MM_TARGET_RADIUS);
            break;
        case MULTIPLE_15MM:
            emit drawMultipleGuide(_points, MULTIPLE_15MM_TARGET_RADIUS);
            break;
        case MULTIPLE_20MM:
            emit drawMultipleGuide(_points, MULTIPLE_20MM_TARGET_RADIUS);
            break;
        case MULTIPLE_SQUARE_20MM:
            emit drawMultipleGuide(_points);
            break;
        case SEMIAUTO_WHITE:
        case SEMIAUTO_GV:
        case AUTO:
            emit drawMultipleGuide(_points);
            break;
        }
    }
    else
    {
        emit drawCross(Point(CENTER_X, CENTER_Y));
    }
}

/***
 * 영상처리가 완료되었다고 CuRAS에게 보내는 함수
 */
void ImageProcessor::sendImageProcessingOk(vector<Point> _points)
{
    static uint8_t  previous_points_nr = 0;

    /* agentCuRAS->getGUIMode() == 2 일때만 동작 */
    if (!agentCuRAS->isSkinTouched() || agentCuRAS->getGUIMode() != TREATMENT_MODE)
    {
        previous_points_nr = 0;
        return;
    }

    /* 포지션의 크기가 보낼 수 있는 크기보다 크면 에러 출력하고 리턴 */
    if (_points.size() > POSTION_MAX)
    {
        qCritical() << "sendImageProcessingOk(): Position Size Overflow...";
        return;
    }

    if (previous_points_nr != _points.size() && !_points.empty())
    {
        agentCuRAS->sendActionPacket(WRITE_IMAGE_PROCESSING_OK, _points.size());
        previous_points_nr = _points.size();
    }
}


void ImageProcessor::doDetect(cv::Mat& _in, cv::Mat& _out)
{
#ifdef ADD_VIEW
    cv::Mat bin;
#endif
    switch (agentCuRAS->getShotMode())
    {
    case SEMIAUTO_WHITE:
        detectorWhite.detect(_in, _out);
#ifdef ADD_VIEW
        bin = detectorWhite.getBinMat();
#endif
        break;
    case SEMIAUTO_GV:
        detectorGV.detect(_in, _out);
        break;
    case AUTO:
        detectorAUTO.detect(_in, _out);
        break;
    }
#ifdef ADD_VIEW
    if (!bin.empty()) {
        emit sig_binImage(bin);
    }
#endif
}


vector<Point> ImageProcessor::getMultipleShotPoints(uint8_t _mode, cv::Mat& _in)
{
    Mat _bin;
    std::vector<cv::Point> _points;
    uint8_t _radius = 0;

    _bin = Mat::zeros(_in.size(), CV_8UC1);

    switch (_mode)
    {
    case MULTIPLE_6MM:
        _radius = MULTIPLE_6MM_TARGET_RADIUS;
        break;
    case MULTIPLE_10MM:
        _radius = MULTIPLE_10MM_TARGET_RADIUS;
        break;
    case MULTIPLE_15MM:
        _radius = MULTIPLE_15MM_TARGET_RADIUS;
        break;
    case MULTIPLE_20MM:
        _radius = MULTIPLE_20MM_TARGET_RADIUS;
        break;
    }

    /* 가상 캔버스(Mat bin)으로 Multiple Target을 그린 후, 포지션 좌표를 구함 */
    if (_radius > 0)
    {
        circle(_bin, Point(CENTER_X, CENTER_Y), _radius, cv::Scalar(255), cv::FILLED);
        _points = OtaNervus::Util::getTargetPotisions(_bin, agentCuRAS->getSpotRadiusSize());
    }

    return _points;
}

#define MARGIN  6
vector<Point> ImageProcessor::getMultiple20MMSquare(uint8_t _overlap)
{
    std::vector<cv::Point> _points;

    int spot_radius = agentCuRAS->getSpotRadiusSize();
    int distance = spot_radius * (100.0-_overlap) / 100;
    int offx = IMAGE_WIDTH / 2 % distance;
    int offy = IMAGE_HEIGHT / 2 % distance;

    for (int y = offy; y < IMAGE_HEIGHT; y += distance*2)
        for (int x = offx; x < IMAGE_WIDTH; x += distance*2)
        {
            if (x-spot_radius < MARGIN || y-spot_radius < MARGIN
                    || x+spot_radius > IMAGE_WIDTH-MARGIN || y+spot_radius > IMAGE_HEIGHT-MARGIN)
                continue;
            _points.push_back(Point(x, y));
        }
    OtaNervus::Util::sortPosition(_points);

    return _points;
}


/***
 * _frame: 들어온 영상 원본 rgb565
 * _in: 들어온 영상을 BGR888로 변경하고, 저장
 * _out: 들어온 영상에 컨투어를 그리기 위한 용도
 */
void ImageProcessor::doImageProcessing(cv::Mat& _in, cv::Mat& _out)
{
    _in.copyTo(_out);

    cv::Mat _bin;

#if 1   // 20230322 JYH Append Detect Circle 깜박임 속도 조절
    if ((++m_nSkipGuide%3) > 0)
    {
        /***
         * OpenCV는 BGR로 처리
         * 따라서 화면에 출력하기 위해 RGB로 변경
         */
        cv::cvtColor(_out, _out, cv::COLOR_BGR2RGB);
        return;
    }
    m_nSkipGuide = 0;
#endif   // 20230322 JYH Append Detect Circle 깜박임 속도 조절

    doCalibration(_in);

    if (!agentCuRAS->isCalibration() && !agentCuRAS->isFootSwitched())
    {
        if (agentCuRAS->isSkinTouched() && !agentCuRAS->isFootSwitched())
        {
            switch (agentCuRAS->getShotMode())
            {
            case SINGLE_NONE:
            case SINGLE_1HZ:
            case SINGLE_2HZ:
                this->points.clear();
                points.push_back(Point(CENTER_X, CENTER_Y));
                break;
            case MULTIPLE_6MM:
            case MULTIPLE_10MM:
            case MULTIPLE_15MM:
            case MULTIPLE_20MM:
                points = getMultipleShotPoints(agentCuRAS->getShotMode(), _in);
                break;
            case MULTIPLE_SQUARE_20MM:
                points = getMultiple20MMSquare(50);     // 20220613 JYH 중첩 수정 30);     /* parameter -> overlap */
                break;

            case SEMIAUTO_WHITE:
            case SEMIAUTO_GV:
            case AUTO:
                doDetect(_in, _bin);

        #ifdef QT_DEBUG
                OtaNervus::Util::drawBinaryContours(_in, _bin, _out, cv::Scalar(0, 255, 0), 2);
        #endif
                points = OtaNervus::Util::getTargetPotisions(_bin, agentCuRAS->getSpotRadiusSize());
                break;
            }
        }

        sendImageProcessingOk(points);
        agentCuRAS->sendPositionToHandpiece(points);
    }

    /***
     * OpenCV는 BGR로 처리
     * 따라서 화면에 출력하기 위해 RGB로 변경
     */
    cv::cvtColor(_out, _out, cv::COLOR_BGR2RGB);

    drawGuide(points);
}


//#define SCALE   10
#define SCALE   20
cv::Mat g_Img_out[10];
uint8_t g_Img_outPos = 0;
void ImageProcessor::setImage(uint8_t* _buffer, uint16_t _size)
{
    static uint8_t  _counter = 0;
    static uint8_t *_Frame_ptr = frame;
#if 0   // 20221007 JYH Delete
    cv::Mat _out;
#endif   // 20221007 JYH Delete

    if (agentCuRAS->getOpMode() != READY)
        return;

    /* ========== 20221007 영상처리 관련 시작 패킷 추가 JYH Append ========================================  */
#if 1   // 20221007 JYH Append
    if ((_buffer == nullptr && _size == 1)) // ||
        //(_buffer == nullptr && _counter != IMAGE_HEIGHT/SCALE && _counter != 0))
    {
        frame_pos++;
        frame_pos %= 2;
        _counter = 0;
        _Frame_ptr = frame + (IMAGE_SIZE*frame_pos);
        return;
    }

    ImagProc_Mutex.lock();
#endif   // 20221007 JYH Append

    /* ========== 20221007 영상처리 관련 시작 패킷 추가 JYH Append ========================================  */
    //if (_buffer == nullptr)
    if (_buffer == nullptr && _size == 0)
    {
        uint8_t *Frame_Temp = frame + (IMAGE_SIZE*frame_pos);
#ifdef QT_DEBUG
        if (_counter != IMAGE_HEIGHT/SCALE && _counter != 0)
            mis_counter++;
#endif

        /* Handpiece에서 x축 라인 한줄 * SCALE로 보냄 */
        if (_counter >= IMAGE_HEIGHT/SCALE)
        {
            /***
             * OpenCV는 BGR로 처리
             * 따라서 입력된 영상(RGB656) -> BGR888로
             */
#if 0   // 20221007 JYH Modify
            _RGB565ToBGR888(srcImageMat, frame);
#else   // 20221007 JYH Append
            _RGB565ToBGR888(srcImageMat, Frame_Temp);
#endif   // 20221007 JYH Append


#if 0   // 20221007 JYH Append
            doImageProcessing(srcImageMat, _out);
            emit SIG_setImage(_out.cols, _out.rows, _out.data);
#else   // 20221007 JYH Append
            doImageProcessing(srcImageMat, g_Img_out[g_Img_outPos]);
            emit SIG_setImage(g_Img_out[g_Img_outPos].cols, g_Img_out[g_Img_outPos].rows, g_Img_out[g_Img_outPos].data);

            g_Img_outPos++;
            g_Img_outPos %= 10;
#endif   // 20221007 JYH Append

#ifdef QT_DEBUG
            frameCnt++;
#endif
        }

        _counter = 0;

#if 1   // 20221007 JYH Append
        ImagProc_Mutex.unlock();
#endif   // 20221007 JYH Append

        return;
    }

    if (_counter < IMAGE_HEIGHT/SCALE && _size == (IMAGE_WIDTH * SCALE * 2))
    {
        //uint8_t *Frame_ptr = frame + (IMAGE_WIDTH * SCALE * 2 *_counter) + (IMAGE_SIZE*frame_pos);
        memcpy(_Frame_ptr, _buffer, _size);
        _Frame_ptr += _size;
        _counter++;
    }
    ImagProc_Mutex.unlock();
}

#define MAX_OFFSET  36*2
#if 1   //20220630 JYH 에이밍 에러체크 10 -> 5번으로 수정
#define RETRY_NR    5
#else
#define RETRY_NR    10
#endif

bool ImageProcessor::isValidPostion(Point& _position)
{
    static Point    _previous;
    static uint8_t  _retry = RETRY_NR;
    bool _ret;

    if (!agentCuRAS->isCalibration())
        return false;

    if (_previous != _position)
    {
        _previous = _position;
        return false;
    }

    _ret = (_position.x != -1 && _position.y != -1);

#if 0
    if (!_ret)
        qDebug("Fail: Search Laser Position: (%d %d), retry: %d", _position.x, _position.y, _retry);
#endif

    _ret = _ret && (abs(CENTER_X-_position.x) < MAX_OFFSET &&  abs(CENTER_X-_position.y) < MAX_OFFSET);
#if 1
    if (!_ret)
        qDebug("Fail: Long Distance Offset: (%d %d), retry: %d", abs(CENTER_X-_position.x), abs(CENTER_X-_position.x), _retry);
#endif

    if (!_ret && (_retry-- <= 0))
    {
#if 1   // 20220623 JYH 에이밍 Fail시 중앙에 레이저 출력하기 위해 수정
        agentCuRAS->setCalibration(false);
        agentCuRAS->sendActionPacket(SET_AIMING_CALIBRATION_FAIL, 1);

        qDebug() << "Invalid AIMING Position";

        _position.x = CENTER_X;
        _position.y = CENTER_Y;
        _retry = RETRY_NR;
        return true;
#else   // 20220623 JYH 에이밍 Fail시 중앙에 레이저 출력하기 위해 수정
        agentCuRAS->setCalibration(false);
        agentCuRAS->sendActionPacket(SET_AIMING_CALIBRATION_FAIL, 1);
        _retry = RETRY_NR;
#endif   // 20220623 JYH 에이밍 Fail시 중앙에 레이저 출력하기 위해 수정

#if 0
        qDebug() << "Invalid Laser Guide Position";
#endif
    }

    if (_ret)
        _retry = RETRY_NR;

    return _ret;
}

void ImageProcessor::doCalibration(cv::Mat& _in)
{
#if 1   // 20220622 JYH 켈리브레이션 하는 소스
    static cv::Mat  _bin;

    if (!agentCuRAS->isCalibration() || !agentCuRAS->isFootSwitched())
        return;

    Point _laserPostion = detectorLaserGuide.getLaserPosition(_in, _bin);

    if (isValidPostion(_laserPostion))
    {
        agentCuRAS->setCalibration(false);
        agentCuRAS->sendActionPacket(WRITE_AIMING_OFF, 1);
        agentCuRAS->sendPosition(points, Point(CENTER_X-_laserPostion.x, CENTER_Y-_laserPostion.y));

#if 1   // 20220705 JYH Append (에이밍 결과 저장)
        QString filename;
        QString filename1;
        QDateTime date = QDateTime::currentDateTime();
        filename = QString::asprintf("%s/%04d-%02d-%02d-%02d%02d%02d",
                         savePathDir.absolutePath().toStdString().c_str(),
                         date.date().year(), date.date().month(), date.date().day(),
                         date.time().hour(), date.time().minute(), date.time().second());

        filename1 = filename + "_Src.jpg";
        imwrite(filename1.toUtf8().constData(), _in);

        filename1 = filename + "_Dst.jpg";
        imwrite(filename1.toUtf8().constData(), _bin);
#endif

    }
#else   // 20220622 켈리브레이션에 실패해도 shot이 나가도록 (오수석님 수정)
    static cv::Mat  _bin;

    if (!agentCuRAS->isCalibration() || !agentCuRAS->isFootSwitched())
        return;

    Point _laserPostion = detectorLaserGuide.getLaserPosition(_in, _bin);

    if (isValidPostion(_laserPostion))
    {
        agentCuRAS->setCalibration(false);
        agentCuRAS->sendActionPacket(WRITE_AIMING_OFF, 1);
        //agentCuRAS->sendPosition(points, Point(CENTER_X-_laserPostion.x, CENTER_Y-_laserPostion.y));
        agentCuRAS->sendPosition(points, Point(0, 0));
    }

#endif
}

/***
 * 주기적으로 원본 Image를 저장하는 쓰레드 (3초마다 저장)
 */
void ImageProcessor::run()
{
    QTimer saveTimer;

    /* init Image Save 디렉토리 */
    //savePathDir = QDir(QCoreApplication::applicationDirPath()+"/Images");
    savePathDir = QDir("C:/VSLS_IMG");
    if (!savePathDir.exists())
        savePathDir.mkdir(".");

    connect(&saveTimer, SIGNAL(timeout()), this, SLOT(saveImage()));

#if 1   // 20220705 JYH Modify (3초 마다 저장)
#define SAVE_PERIOID    3000
#else   // 20220705 JYH Modify (동물시험용 1초 마다 저장)
#define SAVE_PERIOID    1000
#endif   // 20220705 JYH Modify
    saveTimer.start(SAVE_PERIOID);

    exec();
}


void ImageProcessor::saveImage(void)
{
    QString filename;

    if (!agentCuRAS->isSkinTouched())
        return;

    QDateTime date = QDateTime::currentDateTime();

#ifndef SAVE_JPG
    filename.sprintf("%s/%04d-%02d-%02d-%02d%02d%02d_Object.raw",
                     savePathDir.absolutePath().toStdString().c_str(),
                     date.date().year(), date.date().month(), date.date().day(),
                     date.time().hour(), date.time().minute(), date.time().second());

    QFile file(filename);

    /* Trying to open in WriteOnly and Binary mode */
    if(!file.open(QFile::WriteOnly))
    {
        qInfo() << "Could not open image file for writing: " << filename;
        return;
    }

    file.write((const char*)this->frame, ImageSize);
    file.close();
#else
#if 1   // 20220705 JYH Delete
    filename = QString::asprintf("%s/%04d-%02d-%02d-%02d%02d%02d_Object.jpg",
                     savePathDir.absolutePath().toStdString().c_str(),
                     date.date().year(), date.date().month(), date.date().day(),
                     date.time().hour(), date.time().minute(), date.time().second());

    imwrite(filename.toUtf8().constData(), srcImageMat);
#else   // 20220705 JYH Modify (동물시험용)
    filename.sprintf("D:/%04d-%02d-%02d-%02d%02d%02d_Object.jpg",
                     date.date().year(), date.date().month(), date.date().day(),
                     date.time().hour(), date.time().minute(), date.time().second());

    imwrite(filename.toUtf8().constData(), srcImageMat);
#endif
#endif
}

void ImageProcessor::_RGB565ToBGR888(cv::Mat& _in, uint8_t* _data)
{
#pragma unroll (4)
    for (int i = 0; i < IMAGE_WIDTH*IMAGE_HEIGHT; i++)
    {
        uint16_t p = ((uint16_t*)_data)[i];

        int r = (int(p & 0b1111100000000000) >> 11) * 255 / 0b11111;
        int g = (int(p & 0b0000011111100000) >> 5) * 255 / 0b111111;
        int b = int(p & 0b0000000000011111) * 255 / 0b11111;

        _in.data[i * 3] = b;
        _in.data[i * 3 + 1] = g;
        _in.data[i * 3 + 2] = r;
     }
}


#ifdef QT_DEBUG
void ImageProcessor::timeout(void)
{
    if (agentCuRAS->getOpMode() != READY)
        return;

    //qDebug() << "fps :" << frameCnt/10 << "  mis counter: " << mis_counter;
    qDebug() << "fps :" << frameCnt/5 << "  mis counter: " << mis_counter;
    frameCnt = 0;
    mis_counter = 0;
}
#endif
