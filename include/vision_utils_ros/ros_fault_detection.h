#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <vision_utils_ros/frame_container.h>

using namespace cv;
using namespace std;

#ifndef ROSFAULTDETECTION_H_
#define ROSFAULTDETECTION_H_

class ROSFaultDetection{
	public:

		ROSFaultDetection(ros::NodeHandle nh) : is_First_Image_received(false){
			ROS_INFO("ROSFaultDetection Constructor");
			image_sub_ = nh.subscribe("/camera", 1, &ROSFaultDetection::imageCb,this);
			image_pub_ = nh.advertise<sensor_msgs::Image>("Image", 1);

			ros::spin();
		}

		~ROSFaultDetection(){

		}

		void imageCb(const sensor_msgs::ImageConstPtr& msg){
			//mtx_.lock();

			cv_bridge::CvImagePtr cv_ptr;
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			current_.setFrame(cv_ptr->image);
			cv:waitKey(1000);


			if (is_First_Image_received){
				//FaultDetection::run();
			}
	 }
	private:
		MyFrameContainer current_;
		MyFrameContainer last_;
		bool is_First_Image_received;
		ros::Subscriber image_sub_;
		ros::Publisher image_pub_;
};

#endif /* ROSFAULTDETECTION_H_ */
