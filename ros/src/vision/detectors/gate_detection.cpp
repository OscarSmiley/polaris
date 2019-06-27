#include "opencv2/objdetect.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <iostream>
#include <cmath>
#include <chrono>
#include <ros/ros.h>

#include "Detector.hpp"


namespace vision {namespace detectiion {
    class GateDetector : public Detector{
        private:
            struct point_data {
                cv::Point p;
                int n;
            };
            struct detectorResults{
                double r;
                double theta;//Angle from z axis
                double phi;//Angle from x axis
                bool object_found;

            }
            std::vector<point_data> points;
            int radius = 100;
            int wait_time = 20;
            int reset_time = 120;
            double focalLength = 1000; //Specific to camera
            double real_gate_width = 13.87; //Update with real dimensions

            /**
             * Currently Finds x direction of object in relation to camera and distance to object
             **/
            int xDistance(int x, int center_x, double scale_factor)
            {
                int x_dis = abs(x - center_x);/scale_factor;
                if (x - center_x < 0){
                    x_dis = -x_dis;
                }
                return x_dis;
            }
            /**
             * Currently Finds y distance in relation to camera and object distance
             **/
            int yDistance(int y, int center_y, double scale_factor)
            {     
                int y_dis = abs(y - center_y)/scale_factor;
                if (y - center_y > 0){
                    y_dis = -y_dis;
                }
                return y_dis;
            }
            detectorResults toPolar(double x, double y, double z){
                detectorResults result;
                result.r = sqrt(x*x + y*y + z*z);
                result.theta = atan(sqrt(x*x +y*y)/z);
                result.phi = atan(y/x);
                return result;
            }



        public:
            GateDetector(std::string config, const cv::Mat &frame){
                this->frame = frame;
                //Add additional for configs
            }

            //Update to return Detector Result Struct
            detectorResults operator(){

                detectorResults gate_result;
                cv::Mat frame_gray;
                cv::CascadeClassifier object_cascade; 
                object_cascade.load("gate_cascade.xml");//Insert Cascade Classifier Here

                cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
                cv::equalizeHist(frame_gray, frame_gray);
                // Detects objects and stores their location in object
                std::vector<cv::Rect> object;
                object_cascade.detectMultiScale(frame_gray, object, 1.1, 2, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

                //if object is found in frame
                if(!object.empty()) {

                    //Set the current point to the center of the object
                    cv::Point curr_point(object[0].x + object[0].width/2, object[0].y + object[0].height/2);
                    gate_result.object_found = true;
                    //calculate distance to object
                    double z_distance = (real_gate_width*focalLength)/object[0].width;


                    //If best point exists find direction to object.
                    if(curr_point.x){
                        double scale_factor = object[0].width/real_gate_width;
                        //cv::circle(frame, best_point, 50, cv::Scalar(255,255,255), 2, 8, 0); For testing 

                        //Distance accuracy limited currently by Integer type
                        double x_distance = xDistance(best_point.x, frame.cols/2, scale_factor);
                        double y_distance = yDistance(best_point.y, frame.rows/2, scale_factor);
                        gate_result = toPolar(xDistance,yDistance,zDistance);
                    }
                } 
                //If object not found
                else gate_result.object_found = false;

               return gate_results;
            }
    }
}}
















