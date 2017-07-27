#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <vision_utils_ros/frame_container.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/imgproc/imgproc.hpp>

using namespace cv;
using namespace std;

#ifndef ROSFAULTDETECTION_H_
#define ROSFAULTDETECTION_H_

class ROSFaultDetection{
	public:

		ROSFaultDetection(ros::NodeHandle nh, int hessian=400) : current_(), last_(), is_First_Image_received(false),detector_(hessian){
			ROS_INFO("ROSFaultDetection Constructor");
			image_sub_ = nh.subscribe("/camera", 1, &ROSFaultDetection::imageCb,this);
			image_pub_ = nh.advertise<sensor_msgs::Image>("Image", 1);

			ros::spin();
		}

		~ROSFaultDetection(){

		}

		void imageCb(const sensor_msgs::ImageConstPtr& msg){
			cv_bridge::CvImagePtr cv_ptr;
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			current_.setFrame(cv_ptr->image);
			cv:waitKey(1000);

			runFeatureExtractor();

			if (!is_First_Image_received){
				is_First_Image_received = true;
				last_ = current_;
			}
			else{
				run();
			}
	 }

	 void run(){
	     //matcher_.clearing();
	     //matcher_.matchD(first_,second_);
	     //matcher_.separateMatches(first_,second_);
	     //matcher_.getBestMatches(first_,second_);
	     //matcher_.separateBestMatches(first_,second_);

	 }

	 void runFeatureExtractor(){


	   Mat img,descriptors;// = first_.getDescriptors();
	   img = current_.getFrame();
		 std::vector<cv::KeyPoint> k1;// = first_.getKeyPoints();
	   Mat tmp;

	   //fDetector_->detectAndCompute(img,tmp, k1, descriptors);
	   //drawKeypoints(img,k1,tmp);

		 ROS_INFO("here");

		 /*
	   if (descriptors.type()!=CV_32F) {
	     descriptors.convertTo(descriptors, CV_32F);
	   }
		 */
		 ROS_INFO("here");
		 /*


	   current_.setFrame(tmp);
	   current_.setDescriptors(descriptors);
	   current_.setKeyPoints(k1);
		 */
	 }

	private:
		MyFrameContainer current_;
		MyFrameContainer last_;
		FastFeatureDetector detector_;
		bool is_First_Image_received;
		ros::Subscriber image_sub_;
		ros::Publisher image_pub_;
};

#endif /* ROSFAULTDETECTION_H_ */
