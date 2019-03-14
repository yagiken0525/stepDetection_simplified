//
// Created by yagi on 18/12/19.
//

#ifndef SFMDR_FOOTPRINT_MYOPENPOSE_H
#define SFMDR_FOOTPRINT_MYOPENPOSE_H

#include <openpose/headers.hpp>
#include "openPosePerson.h"

class myOpenPose {
public:
    void outputTextFromImage(std::vector<cv::Mat> images, std::string output_Txt_path);
    void outputTextFromVideo(std::string video_path, std::string output_path);
    void DetectTargetPerson(std::vector<OpenPosePerson>& personList, OpenPosePerson& target, cv::Mat image, bool USING_PROB=true, std::string INIT_POINT_FILE="");
    void getPosesInImage(op::Array<float>& poses, std::vector<OpenPosePerson>& personList);
    void getPosesInVideo(cv::VideoCapture& cap, std::vector<std::vector<OpenPosePerson>>& openPoseList);
    void trackingTargetInVideo(cv::VideoCapture& cap, std::vector<std::vector<OpenPosePerson>>& openPoseList);
    void tracking(OpenPosePerson& prevPerson, OpenPosePerson& target, std::vector<OpenPosePerson>& personList);
};


#endif //SFMDR_FOOTPRINT_MYOPENPOSE_H
