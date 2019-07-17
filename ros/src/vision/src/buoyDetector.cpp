#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d.hpp"
#include <stdio.h>

typedef enum 
{
    Jiangshi,
    Aswang,
    Draugr,
    Vetalas
} Buoy_t;

class BuoyDetector  
{
private:
    bool found_buoy = false;
    u_int32_t distance_x;
    cv::VideoCapture cap;
    Buoy_t buoy;
    u_int8_t min_match_count = 10;
    float ratio_thresh = 0.6f; // ratio for Lowe's ratio test
    cv::Mat frame;
    cv::Rect buoy_rect; // rectangle around buoy
    struct Detector {
        cv::Mat buoy_img;
        cv::Ptr<cv::xfeatures2d::SIFT> sift = cv::xfeatures2d::SIFT::create();
        std::vector<cv::KeyPoint> keypoints1, keypoints2;
        cv::Mat descriptors1, descriptors2;
        cv::Ptr<cv::DescriptorMatcher> matcher = 
            cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);

    };
    Detector detector;
public:
    BuoyDetector(const Buoy_t buoy_type, const cv::VideoCapture cap)
    {
        this->cap = cap;
        switch(buoy_type) {
            case Jiangshi:
                this->detector.buoy_img 
                    = cv::imread("../images/Jiangshi_lg.png", cv::IMREAD_GRAYSCALE);
                break;
            case Aswang:
                this->detector.buoy_img 
                    = cv::imread("../images/Aswang_lg.png", cv::IMREAD_GRAYSCALE);
                break;
            case Draugr:
                this->detector.buoy_img 
                    = cv::imread("../images/Draugr_lg.png", cv::IMREAD_GRAYSCALE);
                break;
            case Vetalas:
                this->detector.buoy_img 
                    = cv::imread("../images/Vetalas_lg.png", cv::IMREAD_GRAYSCALE);
                break;
        }
       this->detector.sift->detectAndCompute(this->detector.buoy_img, 
                                            cv::noArray(),
                                            this->detector.keypoints1, 
                                            this->detector.descriptors1); 

        
    }

    bool FindBuoy()
    {
        found_buoy = false;
        cv::Mat gray_frame;
        cap >> frame;

        // Convert the frame to black and white
        cv::cvtColor(frame, gray_frame, cv::COLOR_BGR2GRAY);

        detector.sift->detectAndCompute(gray_frame, 
                                        cv::noArray(), 
                                        detector.keypoints2, 
                                        detector.descriptors2);
        // matching the descriptor vectors with a FLANN based matcher
        std::vector< std::vector<cv::DMatch> > knn_matches;
        detector.matcher->knnMatch( detector.descriptors1, 
                                    detector.descriptors2, 
                                    knn_matches, 
                                    2);

        // Filter matches using the Lowe's ratio test
        std::vector<cv::DMatch> good_matches;
        for (size_t i = 0; i < knn_matches.size(); i++){
            if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance){
                good_matches.push_back(knn_matches[i][0]);
            }
        }
        cv::Mat img_matches;
        if( good_matches.size() >= min_match_count){
            std::vector<cv::Point2f> src_pts;
            std::vector<cv::Point2f> dst_pts;

            for( cv::DMatch x : good_matches){
                src_pts.push_back(detector.keypoints1[x.queryIdx].pt);
                dst_pts.push_back(detector.keypoints2[x.trainIdx].pt);
            }
            cv::Mat H = cv::findHomography(src_pts, dst_pts, cv::RANSAC);
            // get the corners from the jiangshi image
            std::vector<cv::Point2f> obj_corners(4);
            obj_corners[0] = cvPoint(0,0); obj_corners[1] = 
                cvPoint( detector.buoy_img.cols, 0 );
            obj_corners[2] = cvPoint(detector.buoy_img.cols, detector.buoy_img.rows); 
            obj_corners[3] = cvPoint(0, detector.buoy_img.rows);
            std::vector<cv::Point2f> scene_corners(4);
            
            if ( !H.empty() ){
                cv::perspectiveTransform( obj_corners, scene_corners, H);
                cv::Rect rect(scene_corners[0], scene_corners[2]);
                buoy_rect = rect;
                found_buoy = true;
            }
        }
        return found_buoy;
    } 

    cv::Rect GetRect() // returns a cv::Rect if the buoy is found
    {
        if(FindBuoy())
            return buoy_rect;
    }
    
    void Demo()
    {
        if(FindBuoy()){
            cv::rectangle(frame, buoy_rect, cv::Scalar(0, 0, 255), 3);
        }
        cv::imshow("frame", frame); 
    }
}; 
