#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <featuredetection/FD.h>
#include <featuredetection/myfeaturedetection.h>
#include <featuredetection/matcher.h>

using namespace std;

#ifndef ROSFAULTDETECTION_H_

#define ROSFAULTDETECTION_H_


class ROSFaultDetection: FaultDetection{
	public:
		ROSFaultDetection(ros::NodeHandle nh) : FaultDetection(true), is_First_Image_received(false){
			image_pub_ =nh.advertise<sensor_msgs::Image>("image_detection", 1);
			image_sub_ = nh.subscribe("/camera/rgb/image_color", 1, &ROSFaultDetection::imageCb,this);
			ROS_INFO("ROSFaultDetection Constructor");

			ros::spin();
		}

		void startDetection(){
			ROS_INFO("startDetection");
			ROS_INFO("runDetection");
			//cv::imshow("hola", first_.frame_);
		}

		void imageCb(const sensor_msgs::ImageConstPtr& msg){
			//mtx_.lock();
			cv_bridge::CvImagePtr cv_ptr;
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			first_.setFrame(cv_ptr->image);
			runSURF();
			cv_bridge::CvImage out_msg;
			//out_msg.header   = in_msg->header; // Same timestamp and tf frame as input image
			out_msg.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
			out_msg.image    = first_.getFrame(); // Your cv::Mat
			image_pub_.publish(out_msg);

			if (is_First_Image_received){
				//startDetection();
				is_First_Image_received = false;
			}
			else{
				//first_.ORB();
				second_ = first_;
				ROS_INFO("Copy first to second");
				is_First_Image_received = true;
			}
	 }

	private:
		bool is_First_Image_received;
		ros::Subscriber image_sub_;
		ros::Publisher image_pub_;
};

#endif /* ROSFAULTDETECTION_H_ */
