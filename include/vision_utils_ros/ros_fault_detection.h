#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <featuredetection/FD.h>

using namespace cv;
using namespace std;

#ifndef ROSFAULTDETECTION_H_
#define ROSFAULTDETECTION_H_

class ROSFaultDetection{
	public:
		ROSFaultDetection(ros::NodeHandle nh) : is_First_Image_received(false){
			ROS_INFO("ROSFaultDetection Constructor");
			fd_ = FaultDetection(true);
			image_sub_ = nh.subscribe("/camera", 1, &ROSFaultDetection::imageCb,this);
			ros::spin();
		}


		/*
		//Override Methods
		void Matcher::show(std::string window_name){
			cout << "New Show";
		}

		void MyFeatureExtractor::read(VideoCapture v){
		}

		bool FaultDetection::stop(){
		  return true;
		}

		bool FaultDetection::start(){
			return true;
		}
*/
		void startDetection(){
			ROS_INFO("startDetection");
		  //start();
			ROS_INFO("runDetection");
			//cv::imshow("hola", first_.frame_);
			fd_.run();
		  //stop();
		}

		void imageCb(const sensor_msgs::ImageConstPtr& msg){
			//mtx_.lock();
			cv_bridge::CvImagePtr cv_ptr;
			is_First_Image_received = true;
			try{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			}
			catch (cv_bridge::Exception& e){
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
		  }
			fd_.first_.setFrame(cv_ptr->image);

			if (is_First_Image_received){
				startDetection();
			}
	 }
	private:
		bool is_First_Image_received;
		ros::Subscriber image_sub_;
		FaultDetection fd_;
};

#endif /* ROSFAULTDETECTION_H_ */
