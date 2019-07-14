#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <stdio.h>

#include "Distance.hpp" 
#include "Detector.hpp"

typedef enum 
{
    Jiangshi,
    Aswang,
    Draugr,
    Vetalas
} Buoy_t;

class BuoyDetector : public Detector, public Distance
{
private:
    bool FoundBuoy = false;
    u_int32_t distance_x;
    cv::VideoCapture cap;
    Buoy_t buoy;
    struct Detector {
        cv::Mat buoy_img;
        cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    };
    Detector detector;
public:
    BuoyDetector(const Buoy_t buoy_type, const cv::VideoCapture cap)
    {
        this->cap = cap;
        switch(buoy_type) {
            case Jiangshi:
                this->detector.buoy_img = cv::imread("../images/Jiangshi_lg.png", cv::IMREAD_GRAYSCALE);
                break;
            case Aswang:
                this->detector.buoy_img = cv::imread("../images/Aswang_lg.png", cv::IMREAD_GRAYSCALE);
                break;
            case Draugr:
                this->detector.buoy_img = cv::imread("../images/Draugr_lg.png", cv::IMREAD_GRAYSCALE);
                break;
            case Vetalas:
                this->detector.buoy_img = cv::imread("../images/Vetalas_lg.png", cv::IMREAD_GRAYSCALE);
                break;
        }
       this->detector.sift->detectAndCompute(this->detector.buoy_img, cv::noArray(), this->detector.keypoints1, this->detector.descriptors1); 

        
    }
    u_int32_t xDistance();
    u_int32_t yDistance();
    cv::Rect GetRect(); // returns a cv::Rect if the buoy is found
    bool FindBuoy(); // updates the FoundBuoy variable and 
}; 
