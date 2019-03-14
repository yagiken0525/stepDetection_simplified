//
// Created by yagi on 18/01/10.
//

#include "basicFunction.h"
#include <sys/stat.h>

using namespace std;

struct mouseParam {
    int x;
    int y;
};


bool clicked = false;
cv::Point2f clicked_point;


//コールバック関数
void runnerCallBackFunc(int eventType, int x, int y, int flags, void *userdata) {
    switch (eventType) {
        case cv::EVENT_LBUTTONUP:
            std::cout << x << " , " << y << std::endl;
            clicked_point.x = x;
            clicked_point.y = y;
            clicked = true;
    }
}


float bf::sumOfDistOfPoints(vector<cv::Point2f> ptList1, vector<cv::Point2f> ptList2){
//    CV_ASSERT(ptList1.size() == ptList2.size());
    float sumDist = 0;
    for(int i = 0; i < ptList1.size(); i++){
        sumDist += bf::calc2PointDistance(ptList1[i], ptList2[i]);
    }
    sumDist /= ptList1.size();
    return sumDist;
}

void bf::clickPoints(cv::Mat image, vector<cv::Point2f> & clickedPoints, const string file_name) {
    vector<cv::Scalar> colors;
    setColor(&colors);
    ofstream outputTxt(file_name);

    //最初のフレームでスタートラインクリック
    mouseParam mouseEvent;
    string windowName = "Click Target's Face, then push Q to save";
    cv::namedWindow(windowName, CV_WINDOW_AUTOSIZE);
    cv::setMouseCallback(windowName, runnerCallBackFunc, &mouseEvent);

    while (1) {
        cv::imshow(windowName, image);
        int key = cv::waitKey(1);

        if (clicked) {
            clicked = false;
            cv::circle(image, clicked_point, 2, colors[0], 2);
            cv::Point2f pt(clicked_point.x, clicked_point.y);
            clickedPoints.push_back(pt);
            outputTxt << pt.x << " " << pt.y << endl;
        }
        if (key == 'q')
            break;
    }

    cv::destroyAllWindows();
}


string bf::digitString(int num, int digit) {
    char c[32];
    sprintf(c, "%d", num);

    string s(c);
    while (s.length() < digit) {
        s = "0" + s;
    }
    return s.c_str();
}

//string用のsplit関数
vector<string> bf::split(const string &s, char delim) {
    vector<string> elems;
    stringstream ss(s);
    string item;
    while (getline(ss, item, delim)) {
        if (!item.empty()) {
            elems.push_back(item);
        }
    }
    return elems;
}

void bf::setColor(vector<cv::Scalar> *colors) {
    cv::Scalar color1(255, 255, 255);
    cv::Scalar color2(255, 0, 0);
    cv::Scalar color3(0, 255, 0);
    cv::Scalar color4(0, 0, 255);
    cv::Scalar color5(200, 0, 0);
    cv::Scalar color6(0, 200, 0);
    cv::Scalar color7(255, 0, 200);
    cv::Scalar color8(255, 255, 0);
    cv::Scalar color9(0, 255, 255);
    cv::Scalar color10(100, 255, 0);
    cv::Scalar color11(0, 100, 255);

    colors->push_back(color1);
    colors->push_back(color2);
    colors->push_back(color3);
    colors->push_back(color4);
    colors->push_back(color5);
    colors->push_back(color6);
    colors->push_back(color7);
    colors->push_back(color8);
    colors->push_back(color9);
    colors->push_back(color10);
    colors->push_back(color11);

}


void bf::generatePointCloudsIn2Dscale(std::vector<cv::Point2f>& objectCorners, int H, int W, float SCALE, const int AREA_W, const int AREA_H){
    for (int i = 0; i < H; i++) {
        for (int j = 0; j < W; j++) {
            objectCorners.push_back(cv::Point2f((j * SCALE) + AREA_W, (i * SCALE) + AREA_H));
        }
    }
};


template<typename Point>
float bf::calc2PointDistance(Point p1, Point p2) {
    float x_dist = pow((p1.x - p2.x), 2);
    float y_dist = pow((p1.y - p2.y), 2);
    float distance = sqrt((x_dist + y_dist));
    return distance;
}



cv::Point2f bf::warpPoint(cv::Point2f srcPt, cv::Mat H) {

    cv::Point2f warp_pt;

    warp_pt.x =
            H.at<double>(0, 0) * srcPt.x +
            H.at<double>(0, 1) * srcPt.y +
            H.at<double>(0, 2) * 1;

    warp_pt.y =
            H.at<double>(1, 0) * srcPt.x +
            H.at<double>(1, 1) * srcPt.y +
            H.at<double>(1, 2) * 1;

    float z = H.at<double>(2, 0) * srcPt.x +
              H.at<double>(2, 1) * srcPt.y +
              H.at<double>(2, 2) * 1;

    warp_pt.x = warp_pt.x / z;
    warp_pt.y = warp_pt.y / z;

    return warp_pt;
}



void bf::myMkdir(std::string dir) {
    //ディレクトリ作成
    const char *cstr = dir.c_str();
    cout << dir << endl;
    if (mkdir(cstr, 0777) == 0) {
        printf("directory correctly generated\n");
    } else {
        printf("directory already exists\n");
    }
}