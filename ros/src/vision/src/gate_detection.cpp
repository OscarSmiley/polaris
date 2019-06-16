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
            struct detector_results{
                int z_distance;//Distance from camera
                int x_distance;//Right/left
                int y_distance;//up/down
                bool object_found;

            }
            std::vector<point_data> points;
            int radius = 100;
            int wait_time = 20;
            int reset_time = 120;
            double focalLength = 1000; //Specific to camera
            double real_gate_width = 5.5; //Update with real dimensions

            /**
             * Iterates through points to find the point with the largest n. If n is
             * not greater than 3, the point (0,0) is returned.
             */
            cv::Point find_point(std::vector<point_data> points)
            {
                int max = 0;
                int max_index = 0;
                for(int i=0; i<points.size(); i++)
                {
                    if(points.at(i).n > max) {
                        max = points.at(i).n;
                        max_index = i;
                    }
                }
                if(max > 1) return points.at(max_index).p;
                return {0,0};
            }

            /**
             * Iterates through points, adding one to n if the current point is within
             * radius distance. P is then updated to the current point.
             * 
             * If the current point is not within radius of any point in points,
             * a new index is created for it.  
             */
            void add_point(std::vector<point_data> &points, cv::Point point, int radius)
            {
                for(int i=0; i<points.size(); i++)
                {
                    if(sqrt(pow(point.x - points.at(i).p.x, 2) + pow(point.y - points.at(i).p.y, 2)) <= radius) {
                        points.at(i).p.x = point.x;
                        points.at(i).p.y = point.y;
                        points.at(i).n = (points.at(i).n < 6) ? points.at(i).n + 1 : 6;
                        return;
                    }
                }
                points.push_back({point, 1});
            }

            /**
             * Iterates through points and removes one from every n in point_data.
             * If n == 0 in any point_data, remove the data from the vector points. 
             */
            void remove_point(std::vector<point_data> &points)
            {
                for(int i=0; i<points.size(); i++)
                {
                    points.at(i).n = points.at(i).n - 1;
                    if(points.at(i).n == 0) {
                        points.erase(points.begin() + i);
                        i--;
                    }
                }
            }

            /**
             * For internal testing
             */
            void print_points(std::vector<point_data> points)
            {
                for(int i=0; i<points.size(); i++)
                {
                    std::cout << "point: " << points.at(i).p.x << "," << points.at(i).p.y << " num: " << points.at(i).n << std::endl;
                }
                std::cout << "\n";
            }


            /**
             * Currently Finds x direction of object in relation to camera and distance to object
             **/
            int x_distance_to_object(int x, int center_x, int scale_factor)
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
            int y_distance_to_object(int y, int center_y, int scale_factor)
            {     
                int y_dis = abs(y - center_y)/scale_factor;
                if (y - center_y > 0){
                    y_dis = -y_dis;
                }
                return y_dis;
            }



        public:
            GateDetector(std::string config, const cv::Mat &frame){
                this->frame = frame;
                //Add additional for configs
            }

            //Update to return Detector Result Struct
            void operator(){
                detector_results gate_result;
                cv::Mat frame_gray;
                cv::CascadeClassifier object_cascade;

                object_cascade.load(".xml");//Insert Cascade Classifier Here

                cv::cvtColor(frame, frame_gray, cv::COLOR_BGR2GRAY);
                cv::equalizeHist(frame_gray, frame_gray);
                // Detects objects and stores their location in object
                std::vector<cv::Rect> object;
                object_cascade.detectMultiScale(frame_gray, object, 1.1, 2, 0|cv::CASCADE_SCALE_IMAGE, cv::Size(30, 30));

                //if object is found in frame
                if(!object.empty()) {
                    gate_result.object_found = false
                    /** Code for possible false positive detection
                     * cv::Point curr_point(object[0].x + object[0].width/2, object[0].y + object[0].height/2);
                     *add_point(points, curr_point, radius);
                    */
                    gate_result.z_distance = (real_gate_width*focalLength)/object[0].width;
                    cv::Point best_point = find_point(points);
                    //If best point exists find direction to object.
                    if(best_point.x){
                        int scale_factor = object[0].width/real_gate_width;
                        //cv::circle(frame, best_point, 50, cv::Scalar(255,255,255), 2, 8, 0); For testing 

                        //Distance accuracy limited currently by Integer type
                        gate_result.x_distance = x_distance_to_object(best_point.x, frame.cols/2, scale_factor);
                        gate_result.y_distance = y_distance_to_object(best_point.y, frame.rows/2, scale_factor);
                    }
                } 
                //If object not found
                else gate_result.object_found = false;

                // Check if reset_time has passed
                if(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - start).count() >= reset_time) {
                    remove_point(points);
                    start = std::chrono::steady_clock::now();
                }
                /** For testing
                 * if(points.size())print_points(points);
                 * cv::imshow("Window", frame);
                 * cv::waitKey(wait_time);
                */
            }
    }
}}
















