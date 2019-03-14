//
// Created by yagi on 18/12/19.
//

#include "myOpenPose.h"
#include "../basicFunction.h"
#include <sys/stat.h>

using namespace cv;
using namespace std;

void display(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
{
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        cv::imshow("User worker GUI", datumsPtr->at(0).cvOutputData);
        cv::waitKey(1);
    }
    else
        op::log("Nullptr or empty datumsPtr found.", op::Priority::High);
}

void printKeypoints(const std::shared_ptr<std::vector<op::Datum>>& datumsPtr)
{
    if (datumsPtr != nullptr && !datumsPtr->empty())
    {
        cout << datumsPtr->at(0).poseKeypoints.getVolume() << endl;
        std::cout << datumsPtr->at(0).poseKeypoints << std::endl;
    }
    else
        op::log("Nullptr or empty datumsPtr found.", op::Priority::High);
}

void myOpenPose::outputTextFromImage(vector<cv::Mat> images, const std::string output_path){

    //ディレクトリ作成
    const char *cstr = output_path.c_str();
    if (mkdir(cstr, 0777) == 0) {
        printf("directory correctly generated\n");
    } else {
        printf("directory already exists\n");
    }

    ofstream outputfile(output_path + "/human_pose_info.txt");
    op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
    opWrapper.start();

    for(int i=0; i<images.size();i++){
        auto datumProcessed = opWrapper.emplaceAndPop(images[i]);
        if (datumProcessed != nullptr)
        {
            if (datumProcessed != nullptr && !datumProcessed->empty())
            {
                int elem_num = datumProcessed->at(0).poseKeypoints.getVolume();
                for (int j = 0; j < elem_num; j++){
                    if(j % 75 == 0){
                        outputfile << "Person " << (j/75) << " (x, y, score):" << endl;
                    }
                    outputfile << datumProcessed->at(0).poseKeypoints[j] << " ";
                    if(j % 3 == 2){
                        outputfile << endl;
                    }
                }

            }
            else
                op::log("Nullptr or empty datumsPtr found.", op::Priority::High);

            display(datumProcessed);
            cv::imwrite(output_path + bf::digitString(i, 4) + ".jpg", datumProcessed->at(0).cvOutputData);
        }
        else
            op::log("Image could not be processed.", op::Priority::High);

    }
    outputfile.close();
    op::log("Stopping OpenPose...", op::Priority::High);
}


void myOpenPose::outputTextFromVideo(const std::string video_path, const std::string output_path){
    //ディレクトリ作成
    const char *cstr = output_path.c_str();
    if (mkdir(cstr, 0777) == 0) {
        printf("directory correctly generated\n");
    } else {
        printf("directory already exists\n");
    }
    ofstream outputfile(output_path + "/human_pose_info.txt");

    Mat img;
    VideoCapture cap(video_path);
    op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
    opWrapper.start();

    // Process and display image
    int max_frame=cap.get(CV_CAP_PROP_FRAME_COUNT);
    for(int i=0; i<max_frame;i++){
        cap>>img ; //1フレーム分取り出してimgに保持させる
        auto datumProcessed = opWrapper.emplaceAndPop(img);
        if (datumProcessed != nullptr)
        {
            if (datumProcessed != nullptr && !datumProcessed->empty())
            {
                int elem_num = datumProcessed->at(0).poseKeypoints.getVolume();
                for (int j = 0; j < elem_num; j++){
                    if(j % 75 == 0){
                        outputfile << "Person " << (j/75) << " (x, y, score):" << endl;
                    }
                    outputfile << datumProcessed->at(0).poseKeypoints[j] << " ";
                    if(j % 3 == 2){
                        outputfile << endl;
                    }
                }

            }
            else
                op::log("Nullptr or empty datumsPtr found.", op::Priority::High);

            display(datumProcessed);
            cv::imwrite(output_path + bf::digitString(i, 4) + ".jpg", datumProcessed->at(0).cvOutputData);
        }
        else
            op::log("Image could not be processed.", op::Priority::High);

    }
    outputfile.close();
    op::log("Stopping OpenPose...", op::Priority::High);
}


void myOpenPose::DetectTargetPerson(vector<OpenPosePerson>& personList, OpenPosePerson& target, cv::Mat image, bool USING_PROB, string trackingFileName) {
    int peopleNum = int(personList.size());

    //トラッキング対象人物を決定
    int targetID = 0;
    if (USING_PROB) {
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
        float minDist = 10000;
        vector<cv::Point2f> clickPoints;
        string trackingFileName;
        ifstream pointFile(trackingFileName);
        if(pointFile.fail()) {
            bf::clickPoints(image, clickPoints, trackingFileName);
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

void myOpenPose::getPosesInImage(op::Array<float>& poses, vector<OpenPosePerson>& personList){
    int peopleNum = poses.getSize()[0];
    for(int personID = 0; personID < peopleNum; personID++) {
        OpenPosePerson newPerson;
        for(int partID = 0; partID < 25; partID++) {
            cv::Point2f pt(poses[(personID * 75) + (partID * 3)], poses[(personID * 75) + (partID * 3) + 1]);
            newPerson._body_parts_coord.push_back(pt);
            newPerson._probabilityList.push_back(poses[(personID * 75) + (partID * 3)] + 2);
        }
        personList.push_back(newPerson);
    }
}

void myOpenPose::tracking(OpenPosePerson& prevPerson, OpenPosePerson& target, vector<OpenPosePerson>& personList){
    int peopleNum = int(personList.size());
    int trackingID = 0;
    float minDist = 10000;
    for(int personID = 0; personID < peopleNum; personID++) {
        float distOfCoords = bf::sumOfDistOfPoints(personList[personID]._body_parts_coord, prevPerson._body_parts_coord);
        if(distOfCoords < minDist){
            minDist = distOfCoords;
            trackingID = personID;
        }
    }
    personList[trackingID].humanID = 1;
    target = personList[trackingID];
}

void myOpenPose::getPosesInVideo(cv::VideoCapture& cap, std::vector<std::vector<OpenPosePerson>>& openPoseList) {

    //OpenPose
    op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
    opWrapper.start();

    cv::Mat frame;
    cv::Mat firstImage;
    int frameID = 0;

    while (1) {
        if (cap.read(frame)) {
            auto datumProcessed = opWrapper.emplaceAndPop(frame); //OpenPose
            frame = datumProcessed->at(0).cvOutputData;
            op::Array<float> poses = datumProcessed->at(0).poseKeypoints;
            vector<OpenPosePerson> personList;
            if(poses.getSize(0) != 0) {
                this->getPosesInImage(poses, personList);
            }
            cv::resize(frame, frame, cv::Size(), 640.0 / frame.cols, 320.0 / frame.rows);
            cv::imshow("User worker GUI", frame);
            int k = cv::waitKey(1);

            if (k == 27) {
                cap.release();
                cv::destroyAllWindows();
                break;
            }
            openPoseList.push_back(personList);
            frameID++;
        }else{
            break;
        }
    }
    cap.release();
}


void myOpenPose::trackingTargetInVideo(cv::VideoCapture& cap, std::vector<std::vector<OpenPosePerson>>& openPoseList){
    //OpenPose
    op::Wrapper opWrapper{op::ThreadManagerMode::Asynchronous};
    opWrapper.start();
    vector<myOpenPose> personList;
    OpenPosePerson prevTarget;
    OpenPosePerson newTarget;

    cv::Mat frame;
    cv::Mat firstImage;
    int frameID = 0;

    while (1) {
        if (cap.read(frame)) {
            auto datumProcessed = opWrapper.emplaceAndPop(frame); //OpenPose
            frame = datumProcessed->at(0).cvOutputData;
            op::Array<float> poses = datumProcessed->at(0).poseKeypoints;
            vector<OpenPosePerson> personList;
            cv::resize(frame, frame, cv::Size(), 640.0 / frame.cols, 320.0 / frame.rows);
            cv::imshow("User worker GUI", frame);
            int k = cv::waitKey(1);
            getPosesInImage(poses, personList);

            if (!poses.getSize().empty()) { //誰か検出されたら
                if (frameID == 0) {
                    DetectTargetPerson(personList, newTarget ,frame);
                } else {
                    getPosesInImage(poses, personList);
                    tracking(prevTarget, newTarget, personList);
                }
            }
            if (k == 27) {
                cap.release();
                cv::destroyAllWindows();
                break;
            }

            prevTarget = newTarget;
            personList.clear();

            openPoseList.push_back(personList);
            frameID++;
        }
    }
    cap.release();
}

