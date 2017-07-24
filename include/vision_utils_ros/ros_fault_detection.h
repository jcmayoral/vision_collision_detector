#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <featuredetection/FD.h>
#include <mutex>

using namespace cv;
using namespace std;

#ifndef ROSFAULTDETECTION_H_
#define ROSFAULTDETECTION_H_

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

class ROSFaultDetection:public FaultDetection{
	public:
		ROSFaultDetection(ros::NodeHandle nh) : FaultDetection(true), is_First_Image_received(false){
			image_sub_ = nh.subscribe("/camera", 1, &ROSFaultDetection::imageCb,this);
			ros::spin();
		}

		Mat setFrame(){
			return Mat();
		}
		void startDetection(){
		  start();
			run();
		  stop();
		}

		void imageCb(const sensor_msgs::ImageConstPtr& msg){
			//mtx_.lock();
			cv_bridge::CvImagePtr cv_ptr;
			try{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
			}
			catch (cv_bridge::Exception& e){
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
		  }
			first_.setFrame(cv_ptr->image);
			is_First_Image_received = true;

			if (is_First_Image_received){
				startDetection();
			}
	 }
	private:
		bool is_First_Image_received;
		ros::Subscriber image_sub_;
};

#endif /* ROSFAULTDETECTION_H_ */
