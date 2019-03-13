//
// Created by yagi on 18/07/24.
//

#include "FootPrint.h"
#include "basicFunctions/basicFunction.h"
//#include "openPose/myOpenPose.h"
#include <opencv2/sfm.hpp>

using namespace std;
using namespace bf;

int sumVecElem(cv::Vec<unsigned char, CHANNEL> vec);
void getCheckerBoardPoints( vector<cv::Point2f>& imagePoints,  vector<cv::Point2f>& scalePoints,
                            const int W, const int H, const float SCALE, cv::Mat image, const int AREA_W, const int AREA_H);
vector<cv::Point2f> scalingPts(vector<cv::Point2f>& pts, float scale);
void getPosesInImage(op::Array<float>& poses, vector<OpenPosePerson>& personList);
void  VisualizeTarget(OpenPosePerson &target, cv::Mat& image);
cv::Vec3b getPointColor(const int bdID);



/////////////////////////////////////////  CONSTRUCTER  /////////////////////////////////////////////////////



FootPrint::FootPrint(string project_name, string video_name){
    _project_name = project_name;
    _video_name = video_name;
    _projects_path = "../projects/" + _project_name + "/";
    _video_path = _projects_path + "/videos/" + _video_name + VIDEO_TYPE;
//    _openPose_path = "../projects/" + _project_name + "/openPoseData/";
//    _sfm_projects_path = "/home/yagi/sfmDR/projects/" + _project_name + "/";
    _result_folder = _projects_path + "/results/" + video_name + "/";
};



///////////////////////////////////////////  SETUP  ////////////////////////////////////////////////////////////



void FootPrint::setup(){
    getBackGroundImage();
    getHomographyMatrix();
};


void FootPrint::getBackGroundImage() {
    cv::Mat backGround = cv::imread("../projects/" + _project_name + "/backGround.jpg");
    if (backGround.empty()) {
        cv::VideoCapture capture;
        cv::Mat frame;
        if (USE_WEBCAM) {
            capture.open(0); // USBカメラのオープン
        }else{
            cout << _video_path << endl;
            if (capture.open( _projects_path + "/background" + VIDEO_TYPE) == false)
                capture.open( _video_path);
        }
        while (1) {
            capture >> backGround;
            cv::Mat dummy;
            cv::resize(backGround, dummy, cv::Size(), VIS_IMAGE_WIDTH/backGround.cols, VIS_IMAGE_HEIGHT/backGround.rows);
            cv::imshow("checker board: push S to save", dummy);
            int k = cv::waitKey(0);
            if (k == 115) {
                capture.release();
                break;
            }
        }
    }
    cv::imwrite(_projects_path + "/backGround.jpg", backGround);
    this->backGroundImage = backGround;
}


void FootPrint::getHomographyMatrix(){
    vector<cv::Point2f> imagePoints;
    vector<cv::Point2f> scalePoints;
    if(USE_CHECKER_BOARD){
        cv::Mat checkerBoardImage = cv::imread(_projects_path + "/calibrationBoard.jpg");
        if(checkerBoardImage.empty()) {
            cv::VideoCapture capture;
            cv::Mat frame;
            if (USE_WEBCAM) {
                capture.open(0); // USBカメラのオープン
            }else{
                capture.open( _projects_path + "/calibration" + VIDEO_TYPE);
            }
            while (1) {
                capture >> checkerBoardImage;
                cv::Mat dummy;
                cv::resize(checkerBoardImage, dummy, cv::Size(), VIS_IMAGE_WIDTH/checkerBoardImage.cols, VIS_IMAGE_HEIGHT/checkerBoardImage.rows);
                cv::imshow("checker board", dummy);
                int k = cv::waitKey(0);
                if (k == 115) {
                    capture.release();
                    break;
                }
            }
            cv::imwrite(_projects_path + "/calibrationBoard.jpg", checkerBoardImage);
        }

        //SCALE => checker board square size(mm)
        //TARGET_AREA_WIDTH => measuring region(m)
        getCheckerBoardPoints(imagePoints, scalePoints, W, H, SCALE, checkerBoardImage, TARGET_AREA_WIDTH * MtoMM,
                                 TARGET_AREA_HEIGHT * MtoMM);
        for(int i = 0; i<scalePoints.size(); i++){
            scalePoints[i].x+=SCALE;
            scalePoints[i].y+=SCALE;
        }
        scalePoints = scalingPts(scalePoints, PIXEL_SCALE);
        warpH = cv::findHomography(imagePoints, scalePoints);
        cv::warpPerspective(backGroundImage, overViewImage, warpH, cv::Size(TARGET_AREA_WIDTH * 1000 * 2 * PIXEL_SCALE,
                                                                            TARGET_AREA_HEIGHT * 1000 * 2 *
                                                                            PIXEL_SCALE));

    }else{
        selectImagePoints(imagePoints);
        selectWorldPoints(scalePoints);

        warpH = cv::findHomography(imagePoints, scalePoints);
        overViewImSize = cv::Size(scalePoints[2]);
        cv::warpPerspective(backGroundImage, overViewImage, warpH, overViewImSize);
    }
}


void getCheckerBoardPoints( vector<cv::Point2f>& imagePoints,  vector<cv::Point2f>& scalePoints,
                            const int W, const int H, const float SCALE, cv::Mat image, const int AREA_W, const int AREA_H){
    //チェッカーポイントの座標格納
    vector<cv::Point2f> checkerCorners;
    bf::generatePointCloudsIn2Dscale(checkerCorners, H, W, SCALE, AREA_W, AREA_H);
    scalePoints = checkerCorners;
    vector<cv::Point2f> detectedCorners;
    cv::findChessboardCorners(image, cv::Size(W, H), detectedCorners);
    cv::drawChessboardCorners(image, cv::Size(W,H), detectedCorners, true);
    cv::Mat dummy;
    cv::resize(image, dummy, cv::Size(), 640.0/image.cols, 320.0/image.rows);
    imagePoints = detectedCorners;
}


vector<cv::Point2f> scalingPts(vector<cv::Point2f>& pts, float scale){
    vector<cv::Point2f> scalePts;
    for(cv::Point2f pt: pts){
        pt.x *= scale;
        pt.y *= scale;
        scalePts.push_back(pt);
    }
    return scalePts;
}


void FootPrint::selectImagePoints(std::vector<cv::Point2f> & clickedPoints){
    string pointFileName = _projects_path + "/cornerPoints.txt";
    ifstream pointFile(pointFileName);
    if(pointFile.fail()) {
        bf::clickPoints(backGroundImage, clickedPoints, pointFileName);
    }else{
        string str;
        vector<string> strList;
        while (getline(pointFile, str))
        {
            cv::Point2f pt;
            strList = bf::split(str, ' ');
            pt.x = stof(strList[0]);
            pt.y = stof(strList[1]);
            clickedPoints.push_back(pt);
        }
    }
}


void FootPrint::selectWorldPoints(std::vector<cv::Point2f> & scalePoints){
    ifstream scaleFile(_projects_path + "/scale.txt");
    string str;
    vector<string> strList;
//    float imWidth = TARGET_AREA_WIDTH * MtoMM * PIXEL_SCALE;
//    float imHeight = TARGET_AREA_HEIGHT * MtoMM * PIXEL_SCALE;
    while (getline(scaleFile, str))
    {
        cv::Point2f pt;
        strList = bf::split(str, ' ');
        pt.x = stof(strList[0]);
        pt.y = stof(strList[1]);
        pt.x *= PIXEL_SCALE;
        pt.y *= PIXEL_SCALE;
//        pt.x += imWidth;
//        pt.y += imHeight;

        scalePoints.push_back(pt);
    }
}




///////////////////////////////////////////  MAIN PROCESS  ////////////////////////////////////////////////////////////




void FootPrint::run(){
    setup();
    mainProcess();
}


void FootPrint::mainProcess() {
    cv::VideoCapture capture(_video_path);
    cout << _video_path << endl;
    cv::Mat frame;
    int frameNum = capture.get(CV_CAP_PROP_FRAME_COUNT );

    //OpenPose
    op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
    opWrapper.start();
    vector<OpenPosePerson> personList;
    OpenPosePerson prevTarget;
    OpenPosePerson newTarget;

    frameID = 0;
    while (frameID < frameNum) {
        capture.read(frame);
        cout << frameID << " " << !frame.empty() << endl;
        if(!frame.empty()){
            auto datumProcessed = opWrapper.emplaceAndPop(frame);
            frame = datumProcessed->at(0).cvOutputData;
            op::Array<float> poses = datumProcessed->at(0).poseKeypoints;
            if (!poses.getSize().empty()) {
                if (frameID == 0) {
                    firstFrame = frame;
                    DetectTargetPerson(poses, personList, newTarget);
                    Init();
                } else {
                    getPosesInImage(poses, personList);
                    tracking(prevTarget, newTarget, personList);
                    VisualizeTarget(newTarget, frame);
                    EstimateStep(newTarget, frameID);
                }
            }
            frameID++;
        }else
            continue;
        prevTarget = newTarget;
        personList.clear();
        showResult(frame);
    }
    outputResults();
}




///////////////////////////////////////////  TRACKING  ////////////////////////////////////////////////////////////




void FootPrint::tracking(OpenPosePerson& prevPerson, OpenPosePerson& newTarget, vector<OpenPosePerson>& personList){
    int peopleNum = personList.size();
    int trackingID = 0;
    float minDist = 10000;
    for(int personID = 0; personID < peopleNum; personID++) {
        float distOfCoords = sumOfDistOfPoints(personList[personID]._body_parts_coord, prevPerson._body_parts_coord);
        if(distOfCoords < minDist){
            minDist = distOfCoords;
            trackingID = personID;
        }
    }

    personList[trackingID].humanID = 1;
    newTarget = personList[trackingID];
}


void  VisualizeTarget(OpenPosePerson &target, cv::Mat& image){
    for(cv::Point2f pt: target._body_parts_coord){
        cv::circle(image, pt, 4, cv::Scalar(0,0,255), -1);
    }
}


void calculateFootCoM(OpenPosePerson &person){
    cv::Point2f rFoot(0,0);
    cv::Point2f lFoot(0,0);
    lFoot += person._body_parts_coord[19];
    lFoot += person._body_parts_coord[20];
    lFoot += person._body_parts_coord[21];
    rFoot += person._body_parts_coord[22];
    rFoot += person._body_parts_coord[23];
    rFoot += person._body_parts_coord[24];
    person.rFoot = rFoot/3;
    person.lFoot = lFoot/3;
}


void getPosesInImage(op::Array<float>& poses, vector<OpenPosePerson>& personList){
    int peopleNum = poses.getSize()[0];
    for(int personID = 0; personID < peopleNum; personID++) {
        OpenPosePerson newPerson;
        for(int partID = 0; partID < 25; partID++) {
            cv::Point2f pt(poses[(personID * 75) + (partID * 3)], poses[(personID * 75) + (partID * 3) + 1]);
            newPerson._body_parts_coord.push_back(pt);
            newPerson._probabilityList.push_back(poses[(personID * 75) + (partID * 3)] + 2);
        }
        calculateFootCoM(newPerson);
        personList.push_back(newPerson);
    }
}


void FootPrint::DetectTargetPerson(op::Array<float>& poses, vector<OpenPosePerson>& personList, OpenPosePerson& target) {
    int peopleNum = poses.getSize()[0];
    for (int personID = 0; personID < peopleNum; personID++) {
        OpenPosePerson newPerson;
        for (int partID = 0; partID < 25; partID++) {
            cv::Point2f pt(poses[(personID * 75) + (partID * 3)], poses[(personID * 75) + (partID * 3) + 1]);
            newPerson._body_parts_coord.push_back(pt);
            newPerson._probabilityList.push_back(poses[(personID * 75) + (partID * 3)] + 2);
        }
        personList.push_back(newPerson);
    }

    //トラッキング対象人物を決定
    int targetID = 0;

    //1.openposeの関節検出正解確率が最大の人物をトラッキング
    if (TRACKING_MAX_PROB) {
        float maxProb = 0;
        for (int personID = 0; personID < peopleNum; personID++) {
            personList[personID]._probabilityList;
            float sumOfProbability = float(
                    accumulate(personList[personID]._probabilityList.begin(), personList[personID]._probabilityList.end(),
                               0.0));
            if (maxProb < sumOfProbability) {
                targetID = personID;
                maxProb = sumOfProbability;
            }
        }
    }else{
        //2.クリック点に最も近い人物をトラッキング
        float minDist = 10000;
        vector<cv::Point2f> clickPoints;
        string trackingFileName = _projects_path + "/trackingTargetPosition.txt";
        ifstream pointFile(trackingFileName);
        if(pointFile.fail()) {
            bf::clickPoints(firstFrame, clickPoints, trackingFileName);
        }else {
            string str;
            vector<string> strList;
            while (getline(pointFile, str))
            {
                cv::Point2f pt;
                strList = bf::split(str, ' ');
                pt.x = stof(strList[0]);
                pt.y = stof(strList[1]);
                clickPoints.push_back(pt);
            }
        }
        //TODO 複数人のトラッキング
        for (int personID = 0; personID < peopleNum; personID++) {
            float dist = bf::calc2PointDistance(personList[personID]._body_parts_coord[5], clickPoints[0]);
            if(minDist > dist){
                minDist = dist;
                targetID = personID;
            }
        }
    }
    personList[targetID].humanID = 1;
    target = personList[targetID];
}




////////////////////////////////////////////////  ESTIMATE STEP POSITIONS ////////////////////////////////////////////




void FootPrint::Init(){

    for(int i = 0; i < voteMapChannelNum; i++){
        voteMapList.push_back(cv::Mat::zeros(overViewImage.size(), CV_8UC(CHANNEL)));
    }
    stepMap = overViewImage.clone();

    for(int i = 0; i < VISUALIZE_FRAMES; i++) {
        vector<cv::Point2f> ptList;
        stepedPointList.push_back(ptList);
    }

    vector<int> listR;
    vector<int> listL;
    stepNumList.push_back(listR);
    stepNumList.push_back(listL);
    originalStepMap = stepMap.clone();
}


void FootPrint::EstimateStep(OpenPosePerson target, const int imID) {
    voteToStepMap(target, imID, E_RIGHT);
    voteToStepMap(target, imID, E_LEFT);
};


void FootPrint::voteToStepMap(OpenPosePerson target, const int imID, const int footID){
    //Visualize
    int stepedListPointer = imID % VISUALIZE_FRAMES;
    int deleteListPointer = (imID + 1) % VISUALIZE_FRAMES;
    vector<cv::Point2f> visualizeStepList = stepedPointList[stepedListPointer];

    //左右で処理を変更
    cv::Point2f pt = (footID == E_RIGHT ? target.rFoot : target.lFoot);
    vector<cv::Point3f>* stepList = (footID == E_RIGHT ? &RstepList : &LstepList);
    cv::Mat *voteMap = &voteMapList[footID];
    cv::Vec3b color = getPointColor(footID);
    cv::Point2f warpPt = warpPoint(pt, warpH);
    int preStepNum = stepList->size();
    int voteNumSum = 0;

    //近傍VOTE_RANGE分に投票
    int dstChannel = imID % CHANNEL;
    for (int i = 0; i < VOTE_RANGE; i++) {
        for (int j = 0; j < VOTE_RANGE; j++) {
            int xIdx = int(warpPt.x + i - (VOTE_RANGE / 2));
            int yIdx = int(warpPt.y + j - (VOTE_RANGE / 2));
            cv::Point2f stepPt(xIdx, yIdx);

            //投影点がVOTEmap内にあれば
            if (xIdx >= 0 && xIdx < voteMap->cols && yIdx >= 0 && yIdx < voteMap->rows) {
                voteMap->at<cv::Vec<unsigned char, CHANNEL>>(stepPt)[dstChannel] = 1;

                //投票数がしきい値超えていればそのpxを接地点とみなす
                if (sumVecElem(voteMap->at<cv::Vec<unsigned char, CHANNEL>>(stepPt)) >= STEP_THRESHOLD) {
                    stepMap.at<cv::Vec3b>(stepPt) = color;
                    visualizeStepList.push_back(stepPt);
                    stepList->push_back(cv::Point3f(stepPt.x, stepPt.y, imID));
                }
            }
        }
        //voteMapの該当チャンネルを初期化
        resetVoteChannel(dstChannel, voteMap);
    }

    int stepVoteValue = (stepList->size() - preStepNum);
    stepNumList[footID].push_back(stepVoteValue);

    //stepMapから一定フレーム前の接地点を消去
    for (cv::Point2f pt : stepedPointList[deleteListPointer]) {
        stepMap.at<cv::Vec3b>(pt) = originalStepMap.at<cv::Vec3b>(pt);
    }
    stepedPointList[stepedListPointer] = visualizeStepList;
}


int sumVecElem(cv::Vec<unsigned char, CHANNEL> vec){
    int sum = 0.0;
    for(int i = 0; i < CHANNEL; i++){
        sum += vec[i];
    }
    return sum;
}


cv::Vec3b getPointColor(const int bdID){
    switch(bdID){
        case 0:
            return cv::Vec3b(255,0,0);
        case 1:
            return cv::Vec3b(0,255,0);
        case 2:
            return cv::Vec3b(0,0,255);
        case 3:
            return cv::Vec3b(255,0,0);
        case 4:
            return cv::Vec3b(0,255,0);
        case 5:
            return cv::Vec3b(0,0,255);
        default:
            return cv::Vec3b(0,0,0);
    }
}


void FootPrint::resetVoteChannel(const int dstChannel, cv::Mat *voteMap){
    cv::Mat_<cv::Vec<unsigned char, CHANNEL>> voteMapP = *voteMap;
    for (int i = 0; i < stepMap.cols; i++) {
        for (int j = 0; j < stepMap.rows; j++) {
            voteMapP(cv::Point(i , j))[(dstChannel + 1) % CHANNEL] = 0;
        }
    }
}




//////////////////////////////// EXPORT RESULTS ////////////////////////////////////////////////////////////////////




void FootPrint::showResult(cv::Mat frame){
    cv::resize(frame, frame, cv::Size(), VIS_IMAGE_WIDTH/frame.cols, VIS_IMAGE_HEIGHT/frame.rows);
    cv::imshow("input image", frame);
    cv::imshow("step map", stepMap);
    cv::waitKey(1);
}


void FootPrint::outputResults(){
    myMkdir(_result_folder);
    cv::imwrite(_result_folder + "/result.jpg", stepMap);
    exportPointsCSV();
    exportPointsTimeScale();
    exportVoteSomeForPx();
}


void FootPrint::exportPointsCSV() {
    for(int i=E_LEFT; i <= E_RIGHT; i++){
        string fileName = (i == E_RIGHT ? "RstepPoints.csv" : "LstepPoints.csv");
        vector<cv::Point3f> ptList = (i == E_RIGHT ? RstepList : LstepList);
        ofstream file(_result_folder + "/" + fileName);
        for(cv::Point3f pt : ptList){
            file << pt.x * PIXEL_SCALE << " " << pt.y * PIXEL_SCALE << " " << pt.z << endl;
        }
        file.close();
    }
}


void FootPrint::exportPointsTimeScale() {
    for(int i=E_LEFT; i <= E_RIGHT; i++){
        vector<int> stepList = this->stepNumList[i];
        string fileName = (i == E_RIGHT ? "RstepNumList.txt" : "LstepNumList.txt");
        ofstream file(_result_folder + "/" + fileName);
        for(int imID = 0; imID < stepList.size(); imID++) {
            file << stepList[imID] << endl;
        }
        file.close();
    }
}


void FootPrint::exportVoteSomeForPx() {
    string fileName = "voteNum.txt";
    ofstream file(_result_folder + "/" + fileName);
    for(int footID=E_LEFT; footID <= E_RIGHT; footID++){
        cv::Mat *voteMap = &voteMapList[footID];
        for(int i = 0; i < voteMap->rows; i++){
            for(int j = 0; j < voteMap->cols; j++){
                cv::Point2f stepPt(j,i);
                if(sumVecElem(voteMap->at<cv::Vec<unsigned char, CHANNEL>>(stepPt)) >> 0)
                    file << j << " " << i << " " << sumVecElem(voteMap->at<cv::Vec<unsigned char, CHANNEL>>(stepPt)) << endl;
            }
        }
    }
    file.close();
}