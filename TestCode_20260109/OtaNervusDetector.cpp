#include "OtaNervusDetector.h"
#include <opencv2/highgui.hpp>
#include "common.h"
#include <QDir>
#include <QCoreApplication>

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

void Detector::detectLine(Mat& in, Mat& out)
{
    save_dbg_img(in, "in",1);

    // blur
    Mat blur;
    GaussianBlur(in, blur, Size(0, 0), mSigma);
    save_dbg_img(blur, "blur",2);

    // gray scale
    Mat gray;
    cvtColor(blur, gray, COLOR_RGB2GRAY);
    save_dbg_img(gray, "gray",3);

    // threshold
	Mat bin;
    adaptiveThreshold(gray, bin, 255, ADAPTIVE_THRESH_MEAN_C, THRESH_BINARY_INV, mBlock, mC);
     save_dbg_img(bin, "bin",4);

	// erode & dilate
    Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(mKernel, mKernel));
    dilate(bin, bin, kernel, Point(-1, -1), 2);
    erode(bin, bin, kernel, Point(-1, -1), 2);
    //save_dbg_img(kernel, "kernel-erode");
   // result
#if 1
    // find contours
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(bin, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(mRoi.x, mRoi.y));

    // result
    out = Mat::zeros(in.size(), CV_8UC1);
    if (contours.size() && hierarchy.size())
    {
        vector<vector<Point>> validContours;
        for (unsigned int i = 0; i < contours.size(); i++)
        {
            //int child = hierarchy[i][2];  //20220615 JYH warning remove
            int parent = hierarchy[i][3];
//            qDebug("child = %d, parent = %d",child, parent);

            //if (child < 0 && parent < 0)
            if (parent < 0)
            {
                double area = contourArea(contours[i]);

//                qDebug("area = %f\n", area);
                if (!isValidContour(in, contours[i]))
                    continue;

                vector<Point> _contour;
                approxPolyDP(contours[i], _contour, 0.04*arcLength(contours[i], true), true);

                if (area >= mMinArea && area <= mMaxArea && isContourConvex(_contour))
                {
                    validContours.push_back(contours[i]);
                }
            }
        }
        drawContours(out, validContours, -1, Scalar(255), cv::FILLED);
    }

#else
   out = Mat::zeros(in.size(), CV_8UC1);
   vector<vector<Point>> validContours;
   for (int step = 0; step < 2; step++) {
      vector<vector<Point>> contours;
      vector<Vec4i> hierarchy;
      int out_thick = 3;
      if (step) {
         rectangle(bin, Point(0, 0), Point(bin.size().width, bin.size().height), Scalar(255), out_thick);
      }

      // find contours
      findContours(bin, contours, hierarchy, RETR_CCOMP, CHAIN_APPROX_SIMPLE, Point(mRoi.x, mRoi.y));

      if (contours.size() && hierarchy.size()) {
         for (int i = 0; i < contours.size(); i++) {
            int child = hierarchy[i][2];
            int parent = hierarchy[i][3];
            if (child < 0 && parent >= 0) {
               double area = contourArea(contours[i]);
               if (area >= mMinArea && area <= mMaxArea) {
                  if (!step) {
                     validContours.push_back(contours[i]);
                  }
                  else {
                     Rect br = boundingRect(contours[i]);
                     if (br.width < bin.size().width - out_thick && br.height < bin.size().height - out_thick) {
                        validContours.push_back(contours[i]);
                     }
                  }
               }
            }
         }
      }
      if (validContours.size() > 0) {
         break;
      }
   }

   drawContours(out, validContours, -1, Scalar(255), cv::FILLED);
#endif
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


