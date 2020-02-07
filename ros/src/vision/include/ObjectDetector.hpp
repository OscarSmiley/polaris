#ifndef OBJECTDETECTOR
#define OBJECTDETECT

#include <stdio.h>
#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "Detector.hpp"

class ObjectDetector : public Detector
{
private:
    typedef enum
    {
        Jiangshi,
        Aswang,
        Draugr,
        Vetalas
    } Object_t;
    bool found_object = false;
    u_int32_t distance_x;
    CameraInput& camera_input;
    Object_t object;
    u_int8_t min_match_count = 10;
    float ratio_thresh = 0.6f; // ratio for Lowe's ratio test
    float object_width = 60.96f;
    float object_height = 123.19f;
    cv::Rect object_rect; // rectangle around object

    struct Detector {
        cv::Mat object_img;
        cv::Ptr<cv::xfeatures2d::SIFT> sift;
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;
        cv::Ptr<cv::DescriptorMatcher> matcher;
    };
    Detector detector;

public:
    ObjectDetector(CameraInput& input, std::string cascade_name);
    ~ObjectDetector();
    bool update();
    cv::Rect GetRect();
};

#endif
