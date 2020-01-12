#ifndef CAMERAINPUT_HPP
#define CAMERAINPUT_HPP

#include "opencv2/opencv.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"


class CameraInput
{
public:
    CameraInput();
    ~CameraInput() = default;

    bool update();

    const cv::Mat& getFrameFront();

private:
    cv::Mat frame_front;
    cv::VideoCapture input_front;
};

#endif