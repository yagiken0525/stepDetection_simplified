//
// Created by yagi on 18/07/24.
//

#ifndef FOOTPRINT_H
#define FOOTPRINT_H

#include "../src/basicFunctions/basicFunction.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <openpose/core/array.hpp>

#define CHANNEL 200
#define MtoMM 1000

enum EDirection {
    E_LEFT   = 0,
    E_RIGHT  = 1,
};


class FootPrint {
public:
    FootPrint(std::string project_name, std::string video_name);
    ~FootPrint(){};



    ////////////////// PARAMETERS ////////////////////////////////////////////////////////



    // PROCESS
    std::string VIDEO_TYPE = ".mp4"; // 入力動画の拡張子
    bool USE_WEBCAM = false;  // 背景画像取得にwebカメラを使用
    bool TRACKING_MAX_PROB = false;  // openPoseの信頼度を元にトラッキング対象人物を決定

    // HOMOGRAPHY
    bool USE_CHECKER_BOARD= false;  // ホモグラフィー推定にキャリブレーションボードを使用
    int W = 9; // キャリブレーションボード横
    int H = 6; // キャリブレーションボード縦
    float SCALE = 100.0; // キャリブレーションボード1マスの幅(mm)
    float TARGET_AREA_WIDTH = 2; //計測範囲(m)
    float TARGET_AREA_HEIGHT = 1; //計測範囲(m)

    // VOTE
    float PIXEL_SCALE = 0.05; //  1mmあたりのpixel数
    int VOTE_RANGE = 10; //  投票範囲
    int STEP_THRESHOLD = 5; //  接地判定のしきい値

    // VISUALIZATION
    int VISUALIZE_FRAMES = 1000; //  可視化する近傍フレーム数
    float VIS_IMAGE_WIDTH = 640.0; //  出力結果画像幅
    float VIS_IMAGE_HEIGHT = 320.0; // 　出力結果画像高さ




    //////////////////// VARIABLES ////////////////////////////////////////////////////////


    std::string _project_name;
    std::string _video_name;
    std::string _projects_path;
    std::string _video_path;
    std::string _result_folder;
    int frameID;
    cv::Mat firstFrame;
    cv::Size overViewImSize = cv::Size(400, 200);
    cv::Mat warpH;
    std::vector<std::vector<int>> stepNumList;
    int voteMapChannelNum = 2; //右足左足で2
    std::vector<std::vector<cv::Point2f>> stepedPointList;
    std::vector<cv::Point3f> RstepList;
    std::vector<cv::Point3f> LstepList;
    std::vector<cv::Mat> voteMapList;
    cv::Mat stepMap;
    cv::Mat originalStepMap;
    cv::Mat backGroundImage;
    cv::Mat overViewImage;



    //////////////////// METHODS ////////////////////////////////////////////////////////



    void EstimateStep(OpenPosePerson target, const int imID);
    void voteToStepMap(OpenPosePerson target, const int imID, const int footID);
    void selectImagePoints(std::vector<cv::Point2f> & clickedPoints);
    void selectWorldPoints(std::vector<cv::Point2f> & clickedPoints);
    void setup();
    void mainProcess();
    void getBackGroundImage();
    void getHomographyMatrix();
    void run();
    void exportPointsCSV();
    void exportPointsTimeScale();
    void exportVoteSomeForPx();
    void outputResults();
    void tracking(OpenPosePerson& prevPerson, OpenPosePerson& newTarget, std::vector<OpenPosePerson>& personList);
    void resetVoteChannel(const int dstChannel, cv::Mat *voteMap);
    void Init();
    void showResult(cv::Mat frame);
    void DetectTargetPerson(op::Array<float>& poses, std::vector<OpenPosePerson>& personList, OpenPosePerson& target);


};



#endif //FOOTPRINT_H
