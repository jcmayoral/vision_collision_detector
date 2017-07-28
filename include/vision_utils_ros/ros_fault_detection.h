//ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
//OpenCV
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
//vision_utils_ros
#include <vision_utils_ros/frame_container.h>
#include <vision_utils_ros/ros_matcher.h>

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
			Mat input_frame;

			try
			 {
			  //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
				input_frame = cv_bridge::toCvCopy(msg, "bgr8")->image;
				current_.setFrame(input_frame);

			 }
			 catch (cv_bridge::Exception& e)
			 {
			  ROS_ERROR("cv_bridge exception: %s", e.what());
			  return;
			 }

			if (!current_.getFrame().empty()){
				runFeatureExtractor();
			}

			if (!is_First_Image_received){
				is_First_Image_received = true;
				ROS_INFO("First Frame Received");
				last_ = current_;

			}
			else{
				run();
				last_ = current_;
			}
	 }

	 void run(){
	     matcher_.clearing();
	     matcher_.matchD(current_,last_);
	     matcher_.separateMatches(current_,last_);
	     matcher_.getBestMatches(current_,last_);
	     matcher_.separateBestMatches(current_,last_);
			 matcher_.drawBestMatches(current_,last_);
			 publishOutputImage();
	 }

	 void publishOutputImage(){
		 Mat img = matcher_.getFrame();
		 sensor_msgs::Image out_msg; // >> message to be sent
		 cv_bridge::CvImage img_bridge;
		 std_msgs::Header header; // empty header

		 try{
			 img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
			 img_bridge.toImageMsg(out_msg); // from cv_bridge to sensor_msgs::Image
			 image_pub_.publish(out_msg);
		 }

		 catch (cv_bridge::Exception& e){
			 ROS_ERROR("cv_bridge exception: %s", e.what());
			 return;
		 }
/*

		 sensor_msgs::CvBridge bridge_;
		 sensor_msgs::Image::Ptr out_msg = bridge_.cvToImgMsg(&frame, "passthrough");
		 image_pub_.publish(out_msg);
		 */
	 }

	 void runFeatureExtractor(){

	   Mat img,descriptors;// = first_.getDescriptors();
	   img = current_.getFrame();
		 std::vector<cv::KeyPoint> k1;// = first_.getKeyPoints();
	   Mat tmp;

		 detector_.detect( img, k1 );
	   drawKeypoints(img,k1,tmp);
		 extractor_.compute(img, k1, descriptors);

	   if (descriptors.type()!=CV_32F) {
	     descriptors.convertTo(descriptors, CV_32F);
	   }


		 if (!tmp.empty()) {
			 current_.setFrame(tmp);
			 current_.setDescriptors(descriptors);
			 current_.setKeyPoints(k1);
		 }

	 }

	private:
		MyFrameContainer current_;
		MyFrameContainer last_;
		SurfFeatureDetector detector_;
		SurfDescriptorExtractor extractor_;
		ROSMatcher matcher_;
		bool is_First_Image_received;
		ros::Subscriber image_sub_;
		ros::Publisher image_pub_;
};

#endif /* ROSFAULTDETECTION_H_ */
