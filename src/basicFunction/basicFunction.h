//
// Created by yagi on 18/01/10.qqqq
//

#ifndef MAINTEST_BASICFUNCTION_H
#define MAINTEST_BASICFUNCTION_H

#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include "openpose/myOpenPose.h"

namespace bf {

    void myMkdir(std::string dir);

    std::string digitString(int num, int digit);

    std::vector<std::string> split(const std::string &s, char delim);

    void setColor(std::vector<cv::Scalar> *colors);

    template<typename Point>
    float calc2PointDistance(Point p1, Point p2);

    cv::Point2f warpPoint(cv::Point2f srcPt, cv::Mat H);

    float sumOfDistOfPoints(std::vector<cv::Point2f> ptList1, std::vector<cv::Point2f> ptList2);

    void generatePointCloudsIn2Dscale(std::vector<cv::Point2f>& objectCorners, int H, int W, float SCALE, const int  AREA_W, const int AREA_H);

    void clickPoints(cv::Mat image, std::vector<cv::Point2f> & clickedPoints, const std::string filePath="" );

}

#endif //MAINTEST_BASICFUNCTION_H
