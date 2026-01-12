#pragma once
#include <string>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <QDir>
#include "agent_curas.h"

namespace OtaNervus {
	class Detector;
	class Util;
}

class OtaNervus::Detector
{
public:
	enum class Shape {		
		LINE,
        DOT,
        AUTO
	};
	enum class Color {
		WHITE,
		BLACK
	};

	Detector();
	virtual ~Detector();

    void detect(cv::Mat& in, cv::Mat& out);
    cv::Point getLaserPosition(cv::Mat& src, cv::Mat& dst);


	// getter
	Shape getShape() const { return mShape; }
	Color getColor() const { return mColor; }
	cv::Rect getRoi() const { return mRoi; }
	int getSigma() const { return mSigma; }
	int getBlock() const { return mBlock; }
	int getC() const { return mC; }
	int getKernel() const { return mKernel; }
	int getMinArea() const { return mMinArea; }
	int getMaxArea() const { return mMaxArea; }
	int getDotKernel() const { return mDotKernel; }
	// setter
	void setShape(Shape&& shape) { mShape = shape; }
	void setColor(Color&& color) { mColor = color; }
	void setRoi(cv::Rect&& roi) { mRoi = roi; }
	void setSigma(int sigma) { mSigma = sigma; }
	void setBlock(int block) { mBlock = block; }
	void setC(int c) { mC = c; }
	void setKernel(int kernel) { mKernel = kernel; }
	void setMinArea(int minArea) { mMinArea = minArea; }
	void setMaxArea(int maxArea) { mMaxArea = maxArea; }
	void setDotKernel(int kernel) { mDotKernel = kernel; }
    Mat getBinMat() { return m_bin; }

protected:
    void detectLine(cv::Mat& in, cv::Mat& out);
    void detectDot(cv::Mat& in, cv::Mat& out) const;
    void detectAuto(cv::Mat& src, cv::Mat& dst) const;
    //void save_dbg_img(const cv::Mat& img, const char* tag);
    void save_dbg_img(const cv::Mat& img, const char* tag, int index);
    QDir DBGsavePathDir;
    cv::Mat m_bin;
private:
    cv::Mat referenceImg;
    std::vector<cv::Mat> referenceChannel;

	Shape mShape;
	Color mColor;
	cv::Rect mRoi;
	int mSigma;		// blur
	int mBlock;		// adaptive threshold
	int mC;			// adaptive threshold
	int mKernel;	// erode & dilate
	int mMinArea;	// filter by area
	int mMaxArea;	// filter by area
    int mEdgeMinArea;	// filter by area
    int mDotKernel; // erode by dot size

    bool isTarget(cv::Mat& in, std::vector<std::vector<cv::Point>>& conts, int idx);
    bool isValidContour(cv::Mat src, std::vector<cv::Point> _contour) const;
    void equalizeHistogram(cv::Mat src, cv::Mat& dst) const;
};

class OtaNervus::Util
{
private:
	Util() {};
	virtual ~Util() {};

public:
	static void drawBinaryContours(cv::Mat& in, cv::Mat& bin, cv::Mat& out, cv::Scalar color, int thickness=1, int linetype=8);
    static void drawCross(cv::Mat& in, cv::Point _point);
    static void drawGuide(cv::Mat& _in, cv::Point _point);
    static void drawShotOk(cv::Mat& _in, cv::Point _point);
    static void drawSingleGuideAndShotOk(cv::Mat& _in, cv::Point _point, uint8_t _shotOk);
    static void drawMultipleGuideAndShotOk(cv::Mat& _in, std::vector<cv::Point>& _points, uint8_t _shotOk);
    static void sharpenImage(cv::Mat& in);
    //static std::vector<cv::Point> getTargetPotisions(cv::Mat& in, int radius, double overlap=0, int min_area=200);
    //static std::vector<cv::Point> getTargetPotisions(cv::Mat& in, int radius, int overlap=30, int min_cover_ratio=65);    // 20220613 JYH 중첩 50% 수정
    static std::vector<cv::Point> getTargetPotisions(cv::Mat& in, int radius, int overlap=50, int min_cover_ratio=65);
    static void sortPosition(std::vector<cv::Point>& _points);

};
