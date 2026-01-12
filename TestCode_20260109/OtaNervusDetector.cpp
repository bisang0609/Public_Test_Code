#include "OtaNervusDetector.h"
#include <opencv2/highgui.hpp>
#include "common.h"
#include <QDir>
#include <QCoreApplication>

#include <opencv2/imgproc.hpp>

using namespace OtaNervus;
using namespace std;
using namespace cv;

Detector::Detector()
	: mShape(Shape::LINE)
	, mColor(Color::WHITE)
	, mRoi(0, 0, 0, 0)
	, mSigma(6)
	, mBlock(41)
	, mC(0)
	, mKernel(3)
	, mMinArea(100)
	, mMaxArea(numeric_limits<int>().max())
    , mEdgeMinArea(100)
    , mDotKernel(20)
{
    /***
     * 영상의 잔영(비치는 영상)을 제거하기 위한
     * 레퍼런스 이미지
     */
#ifdef ADD_VIEW
    QString sPathSeparator = QString(QDir::separator());
    QString sAppPath = QCoreApplication::applicationDirPath();

    /***
     * 1. [기존] AUTO 모드용 레퍼런스 이미지 로드 (reference2.jpg - 노란 배경)
     */
    if (referenceImg.empty())
    {
        String reference2 = (sAppPath + sPathSeparator + "reference2.jpg").toStdString();
        referenceImg = cv::imread(reference2, IMREAD_COLOR);

        if (!referenceImg.empty())
        {
            cv::cvtColor(referenceImg, referenceImg, cv::COLOR_BGR2Lab);
            GaussianBlur(referenceImg, referenceImg, Size(0, 0), mSigma);
            split(referenceImg, referenceChannel);
        }
    }

    /***
     * 2. [추가] SEMIAUTO 모드용 레퍼런스 이미지 로드 (reference_black.jpg - 검은 배경)
     * 주의: 실행 파일 위치에 'reference_black.jpg' 파일이 있어야 합니다.
     */
    // referenceChannelBlack은 헤더파일에 새로 선언했다고 가정합니다.
    if (referenceChannelBlack.empty())
    {
        // 파일 이름은 실제 저장한 파일명과 똑같이 맞춰주세요
        String referenceBlackPath = (sAppPath + sPathSeparator + "reference_black.jpg").toStdString();
        Mat referenceImgBlack = cv::imread(referenceBlackPath, IMREAD_COLOR);

        if (!referenceImgBlack.empty())
        {
            // 1. Lab 변환
            cv::cvtColor(referenceImgBlack, referenceImgBlack, cv::COLOR_BGR2Lab);

            // 2. [중요/추가] 레퍼런스의 흰색 띠를 더 두껍게 만들기 (Dilation)
            // 커널 사이즈(5,5)를 조절하여 두께를 정합니다. (클수록 많이 덮음)
            Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
            dilate(referenceImgBlack, referenceImgBlack, kernel);

            // 3. 블러링 및 채널 분리
            GaussianBlur(referenceImgBlack, referenceImgBlack, Size(0, 0), mSigma);
            split(referenceImgBlack, referenceChannelBlack);
        }
    }
#else
    if (referenceImg.empty())
    {
        QString sPathSeparator = QString(QDir::separator());
        QString sAppPath = QCoreApplication::applicationDirPath();
        String reference2 = (sAppPath + sPathSeparator + "reference2.jpg").toStdString();
        referenceImg = cv::imread(reference2, IMREAD_COLOR);

        cv::cvtColor(referenceImg, referenceImg, cv::COLOR_BGR2Lab);
        GaussianBlur(referenceImg, referenceImg, Size(0, 0), mSigma);
        split(referenceImg, referenceChannel);
    }
#endif
}

Detector::~Detector()
{
}

void Detector::detect(Mat& in, Mat& out)
{
    switch (mShape)
    {
	case Shape::LINE:
		detectLine(in, out);
		break;
	case Shape::DOT:
		detectDot(in, out);
		break;
    case Shape::AUTO:
        detectAuto(in, out);
        break;
	default:
		out =  Mat::zeros(in.size(), CV_8UC1);
		break;
	}
}

#include <opencv2/highgui.hpp>
#include <qdebug.h>


bool less_by_y(const cv::Point& lhs, const cv::Point& rhs)
{
    return lhs.y < rhs.y;
}

bool less_by_x(const cv::Point& lhs, const cv::Point& rhs)
{
    return lhs.x < rhs.x;
}


#define SCREEN_AREA 35000

#if 1
bool Detector::isValidContour(cv::Mat src, std::vector<cv::Point> _contour) const
{
    // 컨투어의 좌/우, 상/하 최소‧최대 좌표
    auto minmaxX = std::minmax_element(_contour.begin(), _contour.end(), less_by_x);
    auto minmaxY = std::minmax_element(_contour.begin(), _contour.end(), less_by_y);

    // 경계 접촉 여부
    bool touchesLeftRight = (minmaxX.first->x <= 2 ||
                             minmaxX.second->x >= src.cols - 2);
    bool touchesTopBottom = (minmaxY.first->y <= 2 ||
                             minmaxY.second->y >= src.rows - 2);

    if (mShape == Shape::LINE) {
        // LINE 모드에서는 전체 영상 면적 대비 비율을 사용하여 작은 병변을 허용
        double area_ratio = (double)cv::contourArea(_contour) /
                            (src.cols * src.rows) * 100.0;
        const double boundaryThreshold = 30.0; // 경계 접촉 시 최소 면적 비율 (%)

        // 하나라도 경계에 닿고 면적 비율이 작으면 무시
        if ((touchesLeftRight || touchesTopBottom) &&
            area_ratio < boundaryThreshold) {
            return false;
        }
        // 좌우와 상하 모두 닿는 경우(거의 전체 화면)는 제외
        //if (touchesLeftRight && touchesTopBottom) {
        //   return false;
        //}
    } else {
        // 다른 모드(DOT/AUTO)에서는 기존 조건을 유지
        double area_ratio = cv::contourArea(_contour) / SCREEN_AREA * 100.0;

        if (minmaxX.first->x <= 2 || minmaxX.second->x >= src.cols - 2) {
            if (area_ratio < 30.0)  // 전체 면적의 30% 미만이면 제외
                return false;
        }
        if (minmaxY.first->y <= 2 || minmaxY.second->y >= src.rows - 2) {
            if (area_ratio < 30.0)
                return false;
        }
        if (minmaxX.first->x <= 2 && minmaxX.second->x >= src.cols - 2 &&
            minmaxY.first->y <= 2 && minmaxY.second->y >= src.rows - 2) {
            return false;
        }
    }
    return true;
}

#else
bool Detector::isValidContour(Mat src, vector<Point> _contour) const
{
    auto minmaxX = minmax_element(_contour.begin(), _contour.end(), less_by_x);
    auto minmaxY = minmax_element(_contour.begin(), _contour.end(), less_by_y);

    double area_ratio = contourArea(_contour)/SCREEN_AREA*100;

    if (minmaxX.first->x <= 2 || minmaxX.second->x >= src.size().width-2)
    {
        if (area_ratio < 30.0)    /* 전체 면적읜 30% 이상 */
            return false;
    }

    if (minmaxY.first->y <= 2 || minmaxY.second->y >= src.size().height-2)
    {
        if (area_ratio < 30.0)
            return false;
    }

    if (minmaxX.first->x <= 2 && minmaxX.second->x >= src.size().width-2)
    {
        if (minmaxY.first->y <= 2 && minmaxY.second->y >= src.size().height-2)
            return false;
    }

    return true;
}
#endif
bool Detector::isTarget(Mat& in, std::vector<std::vector<cv::Point>>& conts, int idx)
{
    Mat mask = cv::Mat::zeros(in.size(), CV_8UC1);
    cv::drawContours(mask, conts, idx, 255, FILLED);

    cv::Scalar mean = cv::mean(in, mask);

#if 0
    double dist[3];

    dist[0] = sqrt(pow(mean[0]-255, 2) + pow(mean[1], 2) + pow(mean[2], 2));
    dist[1] = sqrt(pow(mean[0], 2) + pow(mean[1]-255, 2) + pow(mean[2], 2));
    dist[2] = sqrt(pow(mean[0], 2) + pow(mean[1], 2) + pow(mean[2]-255, 2));

    qDebug("dist[0] %f, dist[1] %f, dist[2] %f", dist[0], dist[1], dist[2]);
#endif
    qDebug("mean[0] %f, mean[1] %f, mean[2] %f\n", mean[0], mean[1], mean[2]);

//    return dist[0] < dist[1] && dist[0] < dist[2];
    return mean[2] < 80;
}

void Detector::save_dbg_img(const cv::Mat& img, const char* tag, int index)
{
    if (img.empty())
        return;

    // 저장 폴더 설정
    DBGsavePathDir = QDir("C:/VSLS_IMG/Debug");
    if (!DBGsavePathDir.exists()) {
        DBGsavePathDir.mkpath(".");
    }

    QDateTime date = QDateTime::currentDateTime();

    QString filename = QString::asprintf(
        //"%s/%04d-%02d-%02d-%02d%02d%02d_%s.jpg",
        //"%s/%d-%02d-%02d-%02d%02d%02d_%s.jpg",
        "%s/%02d%02d%02d_%02d_%02d_%02d_%s.jpg",
        DBGsavePathDir.absolutePath().toStdString().c_str(),
        //date.date().year(),
        date.time().hour(),
        date.time().minute(),
        date.time().second(),
        index,
        date.date().month(),
        date.date().day(),
        (tag && *tag) ? tag : "DBG"
        );

    cv::imwrite(filename.toStdString(), img);
}

// 예: 띠 높이보다 큰 값(단위: 픽셀), 상황에 따라 조정하세요.
static const int kTophatKernelSize = 31;  // 타원형 커널 크기 (31×31)
static const int kCloseKernelSize  = 15;  // 클로징 커널 크기  (15×15)
void Detector::detectLine(Mat& in, Mat& out)
{
    save_dbg_img(in, "in", 1);

    // 1. 블러 -> 그레이 변환
    cv::Mat blur;
    GaussianBlur(in, blur, cv::Size(0, 0), mSigma);

    cv::Mat gray;
    cvtColor(blur, gray, cv::COLOR_RGB2GRAY);

    // 2. white‑tophat 전처리로 세로 띠 제거
    Mat ellKernel = getStructuringElement(MORPH_ELLIPSE,
                                          Size(kTophatKernelSize, kTophatKernelSize));
    Mat opened, tophat;
    morphologyEx(gray, opened, MORPH_OPEN, ellKernel);
    tophat = gray - opened;
    save_dbg_img(tophat, "tophat", 2);
    // 3. 적응형 이진화 (tophat 이미지를 사용)
    cv::Mat bin;
    cv::adaptiveThreshold(tophat, bin, 255,
                          cv::ADAPTIVE_THRESH_MEAN_C, cv::THRESH_BINARY_INV,
                          mBlock, mC);
    save_dbg_img(bin, "bin", 3);
#ifdef ADD_VIEW
    m_bin = bin.clone();
#endif
    // 4. 기본 모폴로지 연산 (노이즈 제거)
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE,
                                               cv::Size(mKernel, mKernel));
    cv::dilate(bin, bin, kernel, cv::Point(-1, -1), 2);
    cv::erode(bin, bin, kernel, cv::Point(-1, -1), 2);

    // 5. 클로징으로 띠로 끊어진 경계 연결
    Mat closeKernel = getStructuringElement(MORPH_ELLIPSE,
                                            Size(kCloseKernelSize, kCloseKernelSize));
    morphologyEx(bin, bin, MORPH_CLOSE, closeKernel, Point(-1,-1), 2);
    // 6. 컨투어 추출 및 필터링 (기존 로직 유지)
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(bin, contours, hierarchy, cv::RETR_CCOMP,
                     cv::CHAIN_APPROX_SIMPLE, cv::Point(mRoi.x, mRoi.y));

    out = cv::Mat::zeros(in.size(), CV_8UC1);
    if (!contours.empty() && !hierarchy.empty())
    {
        std::vector<std::vector<cv::Point>> validContours;
        for (size_t i = 0; i < contours.size(); ++i)
        {
            if (hierarchy[i][3] < 0)
            {
                double area = cv::contourArea(contours[i]);
                if (!isValidContour(in, contours[i]))
                    continue;

                // 세로 띠(길쭉한 직사각형) 제외
                cv::RotatedRect rotRect = cv::minAreaRect(contours[i]);
                float w = rotRect.size.width;
                float h = rotRect.size.height;
                float ratio = (w < h) ? (w / h) : (h / w);
                bool isVerticalStripe = (ratio < 0.25f);
                //bool isVerticalStripe = (ratio < 0.1f);
                // 기존 면적/원형도/사각형 필터링...
                // (필요하면 기존 조건들을 그대로 추가하세요)
                qDebug() << "Contour area:" << area << " ratio:" << ratio
                         << " verticalStripe:" << isVerticalStripe;
                if (area >= mMinArea && area <= mMaxArea && !isVerticalStripe)
                {
                    validContours.push_back(contours[i]);
                }
            }
        }
        cv::drawContours(out, validContours, -1, cv::Scalar(255),
                         cv::FILLED);
    }
}

void Detector::detectDot(Mat& in, Mat& out) const
{
	// blur
	Mat blur;
    GaussianBlur(in, blur, Size(0, 0), mSigma);

	// split plans
	Mat plans[3];
	split(blur, plans);

	// threshold
	Mat bins[3];
    for (int i = 0; i < 3; i++) {
		adaptiveThreshold(plans[i], bins[i], 255, ADAPTIVE_THRESH_MEAN_C, mColor == Color::WHITE ? THRESH_BINARY : THRESH_BINARY_INV, mBlock, mC);

        // erode & dilate
		Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(mKernel, mKernel));
        erode(bins[i], bins[i], kernel);
        dilate(bins[i], bins[i], kernel);
	}

	Mat bin;
    bitwise_or(bins[0], bins[1], bin);
    bitwise_or(bins[2], bin, bin);

	// find contours
	vector<vector<Point>> contours;
	vector<Vec4i> hierarchy;
	findContours(bin, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(mRoi.x, mRoi.y));

	// result
	out = Mat::zeros(in.size(), CV_8UC1);
	if (contours.size() && hierarchy.size()) {
		vector<Point> points;
        for (unsigned int i = 0; i < contours.size(); i++) {
			int child = hierarchy[i][2];
			int parent = hierarchy[i][3];
			if (child < 0 && parent < 0) {
				double area = contourArea(contours[i]);
                if (area >= mMinArea && area <= mMaxArea)
                {
					Moments m = moments(contours[i]);
					if (m.m00 != 0) {
						points.push_back(Point(int(m.m10 / m.m00), int(m.m01 / m.m00)));
					}
				}
			}
		}
		if (!points.empty()) {
			vector<Point> hull;
			convexHull(points, hull);
			fillConvexPoly(out, hull, Scalar(255));
			Mat kernel = getStructuringElement(MORPH_ERODE, Size(mDotKernel, mDotKernel));
			erode(out, out, kernel);
		}
	}
}

void Detector::detectAuto(Mat& src, Mat& dst) const
{
    Mat srcLab;
    cv::cvtColor(src, srcLab, cv::COLOR_BGR2Lab);
    GaussianBlur(srcLab, srcLab, Size(0, 0), mSigma);

    std::vector<cv::Mat> channels;
    split(srcLab, channels);

    absdiff(channels[0], referenceChannel[0], channels[0]);

    Mat blur;
    GaussianBlur(channels[0], blur, Size(0, 0), mSigma);

    Mat thresh;
    threshold(blur, thresh, -1, 255, THRESH_BINARY+THRESH_OTSU);

    Mat threshEx;
    adaptiveThreshold(blur, threshEx, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY, mBlock, mC);

    bitwise_and(thresh,threshEx, thresh);

    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(mKernel, mKernel));
    dilate(thresh, thresh, kernel, Point(-1, -1), 2);
    erode(thresh, thresh, kernel, Point(-1, -1), 2);
//    dilate(thresh, thresh, kernel);

    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(thresh, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

    dst = Mat::zeros(src.size(), CV_8UC1);
    if (contours.size() && hierarchy.size()) {
        vector<vector<Point>> validContours;
        for (unsigned int i = 0; i < contours.size(); i++) {
            //int child = hierarchy[i][2];      //20220615 JYH warning remove
            //int parent = hierarchy[i][3];     //20220615 JYH warning remove
//            qDebug("child = %d, parent = %d",child, parent);
// FIXME            if (child < 0 && parent < 0) {
            {
                double area = contourArea(contours[i]);
//                qDebug("area = %f\n", area);

                if (contours.size() != 1 && !isValidContour(src, contours[i]))
                    continue;

                if (area >= mMinArea && area <= mMaxArea) {
                    validContours.push_back(contours[i]);
                }
            }
        }
        drawContours(dst, validContours, -1, Scalar(255), cv::FILLED);
    }
}


Point Detector::getLaserPosition(Mat& src, Mat& dst)
{
    Mat img_hsv;
    cvtColor(src, img_hsv, cv::COLOR_BGR2Lab);

    vector<Mat> channels;
    split(img_hsv, channels);

    Mat blur;
    GaussianBlur(channels[0], blur, Size(0, 0), mSigma);

    Mat thresh;
#if 1   //20220622 OJH 스레쉬홀드 값 수정(에이밍 빔 못잡는 문제)
    threshold(blur, thresh, 220, 255, THRESH_BINARY_INV);
#else
    //threshold(blur, thresh, 210, 255, THRESH_BINARY_INV);
    threshold(blur, thresh, 215, 255, THRESH_BINARY_INV);
#endif
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(thresh, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE);

    src.copyTo(dst);
    //dst = src;  //Mat::zeros(src.size(), CV_8UC1);
    if (contours.size() && hierarchy.size()) {
        vector<vector<Point>> validContours;
        for (unsigned int i = 0; i < contours.size(); i++) {
            int child = hierarchy[i][2];
            int parent = hierarchy[i][3];
            if (child < 0 && parent == 0)
            {
//                if (!isValidContour(src, contours[i]))
//                  continue;

                double area = contourArea(contours[i]);
                if (area >= mMinArea && area <= mMaxArea) {
//                    qDebug("getLaserPostion Area = %f\n", area);
                    validContours.push_back(contours[i]);
                }
#if 0
                else
                {
                    qDebug("Invalid Laser Area: %d -> Min(%d) Max(%d)", (int)area, (int)mMinArea, (int)mMaxArea);
                }
#endif
            }
        }

#ifdef QT_DEBUG
        drawContours(dst, validContours, -1, Scalar(255), cv::FILLED);
#endif

        if (validContours.size() >= 1)
        {
            cv::Moments M;
            double area = 0.0;
            for (unsigned int i = 0; i < validContours.size(); i++)
            {
                if (contourArea(validContours[i]) > area)
                {
                    area = contourArea(validContours[i]);
                    M = cv::moments(validContours[i]);
                }
            }
#if 1   // 20220705 JYH Append (에이밍 결과 저장)
            //circle(dst, Point(M.m10/M.m00+(CENTER_X-50), M.m01/M.m00+(CENTER_Y-50)), 10, Scalar(255), -1, 8, 0);
            circle(dst, Point(M.m10/M.m00, M.m01/M.m00), 10, Scalar(0,255,0), 2, 8, 0);
#endif
            return cv::Point(M.m10/M.m00, M.m01/M.m00);
        }
        else
            return cv::Point(-1, -1);
    }

    return cv::Point(-1, -1);
}

// FIXME: do not use
void Detector::equalizeHistogram(Mat src, Mat& dst) const
{
    int R[256] = {0};
    int G[256] = {0};
    int B[256] = {0};
    int sum = src.rows * src.cols;

    for (int i = 0; i < src.rows; i++) {
        for (int j = 0; j < src.cols; j++) {
            B[src.at<Vec3b>(i, j)[0]]++;
            G[src.at<Vec3b>(i, j)[1]]++;
            R[src.at<Vec3b>(i, j)[2]]++;
        }
    }

    double val[3] = {0};
    for (int i = 0; i < 256; i++) {
        val[0] += B[i];
        val[1] += G[i];
        val[2] += R[i];
        B[i] = val[0] * (256-1) / sum;
        G[i] = val[1] * (256-1) / sum;
        R[i] = val[2] * (256-1) / sum;
    }

    dst = Mat::zeros(src.rows, src.cols, CV_8UC3);
    for(int i = 0; i < src.rows; i++){
        for(int j = 0; j < src.cols; j++){
            dst.at<Vec3b>(i, j)[0] = B[src.at<Vec3b>(i, j)[0]];
            dst.at<Vec3b>(i, j)[1] = B[src.at<Vec3b>(i, j)[1]];
            dst.at<Vec3b>(i, j)[2] = B[src.at<Vec3b>(i, j)[2]];
        }
    }
}

#ifdef QT_DEBUG
void Util::drawBinaryContours(Mat& in, Mat& bin, Mat& out, Scalar color, int thickness, int linetype)
{
	if (&in != &out) {
		in.copyTo(out);
	}
    Mat copyBin;
    // FIX ME:
    bin.copyTo(copyBin);
	vector<vector<Point>> contours;
    findContours(copyBin, contours, RETR_LIST, CHAIN_APPROX_SIMPLE);
    drawContours(out, contours, -1, color, thickness, linetype);
}
#endif


#if 1
/****************************************************************************************
* in : 바이너리 이미지.
* radius : 레이저 반지름.
* overlap : 레이저 겹치는 정도. 0~99(%)
* min_cover_ratio : 레이저가 커버할 모반의 레이저 면적 대비 비율. 0~100(%)
*/
#define MARGIN 20

vector<Point> Util::getTargetPotisions(Mat& in, int radius, int overlap, int min_cover_ratio)
{
   vector<Point> res;

   int width = in.size().width;
   int height = in.size().height;

   vector<vector<Point>> contours;
   findContours(in, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
   if (contours.size() == 0)
      return res;

   sort(contours.begin(), contours.end(), [](vector<Point>& first, vector<Point>& second)->bool
      {
         Rect first_rect = boundingRect(first);
         Rect second_rect = boundingRect(second);
         return first_rect.y < second_rect.y || (first_rect.y == second_rect.y && first_rect.x < second_rect.x);
      });

   for (int i = 0; i < (int)contours.size(); i++) {
      vector<Point> points;

      Mat nervus = Mat::zeros(in.size(), CV_8UC1);
      drawContours(nervus, contours, i, Scalar(255), FILLED, 8);

      Moments M = moments(contours[i]);
      Point center(M.m10 / M.m00, M.m01 / M.m00);

      overlap = max(0, min(99, overlap));
      int distance = radius * 2 * (100.0 - overlap) / 100;
      int offx = center.x % distance;
      int offy = center.y % distance;

      vector<Point> base;
      for (int x = offx; x < width; x += distance) {
         for (int y = offy; y < height; y += distance) {
            if (x < MARGIN || y < MARGIN || x > width - MARGIN || y > height - MARGIN)
               continue;
            base.push_back(Point(x, y));
         }
      }

      /*int half_distance = distance / 2;       // 20220613 JYH 삭제 중첩 그리는 부분
      offx = offx < half_distance ? offx + half_distance : offx - half_distance;
      offy = offy < half_distance ? offy + half_distance : offy - half_distance;
      for (int x = offx; x < width; x += distance) {
         for (int y = offy; y < height; y += distance) {
            if (x < MARGIN || y < MARGIN || x > width - MARGIN || y > height - MARGIN)
               continue;
            base.push_back(Point(x, y));
         }
      }*/

      struct AREA {
         AREA(Point _p, int _area) { p = _p; area = _area; }
         Point p;
         int area;
      };

      double pi = 3.14159265358979323846;
      int min_area = pi * radius * radius * min_cover_ratio / 100;
      vector<AREA> area;
      Mat last;
      int last_area = countNonZero(nervus);
      for (vector<Point>::iterator p = base.begin(); p != base.end(); p++) {
         nervus.copyTo(last);
         circle(last, *p, radius, 0, FILLED);
         int cur_area = last_area - countNonZero(last);
         if (cur_area >= min_area)
            area.push_back(AREA(*p, cur_area));
      }
      sort(area.begin(), area.end(), [](AREA& first, AREA& second)->bool
         {
            return first.area > second.area;
         });

      nervus.copyTo(last);
      for (vector<AREA>::iterator a = area.begin(); a != area.end(); a++) {
         Point p = a->p;
         if (nervus.at<uint8_t>(p) > 0) {
            points.push_back(p);
            circle(last, p, radius, 0, FILLED);
         }
         else {
            Mat tmp;
            last.copyTo(tmp);
            circle(tmp, p, radius, 0, FILLED);
            int diff_area = countNonZero(last) - countNonZero(tmp);
            if (diff_area >= min_area) {
               points.push_back(p);
               last = tmp;
            }
         }
      }

      sort(points.begin(), points.end(), [](Point& first, Point& second)->bool
         {
            return first.y < second.y || (first.y == second.y && first.x < second.x);
         });

      res.insert(res.end(), points.begin(), points.end());
   }

   return res;
}
#else
/****************************************************************************************
* in : 바이너리 이미지.
* radius : 레이저 반지름.
* overlap : 레이저 겹치는 정도. 0~99(%)
* min_cover_ratio : 레이저가 커버할 모반의 레이저 면적 대비 비율. 0~100(%)
*/

#define MARGIN  20      /* 영역 내의 포인트만 처리하기 위한 마진 */

vector<Point> Util::getTargetPotisions(Mat& in, int radius, int overlap, int min_cover_ratio)
{
   vector<Point> res;

   int width = in.size().width;
   int height = in.size().height;

   vector<vector<Point>> contours;
   findContours(in, contours, RETR_EXTERNAL, CHAIN_APPROX_NONE);
   if (contours.size() == 0)
      return res;

   Moments M = moments(contours[0]);
   Point center(M.m10 / M.m00, M.m01 / M.m00);

   overlap = max(0, min(99, overlap));
   int distance = radius * 2 * (sqrt(2) * 100 - overlap) / 100;
   int offx = center.x % distance;
   int offy = center.y % distance;

   vector<Point> base;
   for (int x = offx; x < width; x += distance) {
      for (int y = offy; y < height; y += distance) {
          if (x < MARGIN || y < MARGIN || x > width-MARGIN || y > height-MARGIN)
              continue;
          base.push_back(Point(x, y));
      }
   }

   int half_distance = distance / 2;
   offx = offx < half_distance ? offx + half_distance : offx - half_distance;
   offy = offy < half_distance ? offy + half_distance : offy - half_distance;
   for (int x = offx; x < width; x += distance) {
      for (int y = offy; y < height; y += distance) {
          if (x < MARGIN || y < MARGIN || x > width-MARGIN || y > height-MARGIN)
              continue;
          base.push_back(Point(x, y));
      }
   }

   struct AREA {
      AREA(Point _p, int _area) { p = _p; area = _area; }
      Point p;
      int area;
   };

   double pi = 3.14159265358979323846;
   int min_area = pi * radius * radius * min_cover_ratio / 100;
   vector<AREA> area;
   Mat last;
   int last_area = countNonZero(in);
   for (vector<Point>::iterator p = base.begin(); p != base.end(); p++) {
      in.copyTo(last);
      circle(last, *p, radius, 0, FILLED);
      int cur_area = last_area - countNonZero(last);
      if (cur_area >= min_area)
         area.push_back(AREA(*p, cur_area));
   }
   sort(area.begin(), area.end(), [](AREA& first, AREA& second)->bool
      {
         return first.area > second.area;
      });

   in.copyTo(last);
   for (vector<AREA>::iterator a = area.begin(); a != area.end(); a++) {
      Point p = a->p;
      if (in.at<uint8_t>(p) > 0) {
         res.push_back(p);
         circle(last, p, radius, 0, FILLED);
      }
      else {
         Mat tmp;
         last.copyTo(tmp);
         circle(tmp, p, radius, 0, FILLED);
         int diff_area = countNonZero(last) - countNonZero(tmp);
         if (diff_area >= min_area) {
            res.push_back(p);
            last = tmp;
         }
      }
   }

   sortPosition(res);

   return res;
}
#endif

void Util::sortPosition(std::vector<cv::Point>& _points)
{
    // 결과를 x, y 기준으로 정렬
    sort(_points.begin(), _points.end(), [](Point& first, Point& second)->bool
        {
            return first.y < second.y || (first.y == second.y && first.x < second.x);
        });
}


