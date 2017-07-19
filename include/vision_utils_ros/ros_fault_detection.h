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
  cout << "my new start" << endl;
}

class ROSFaultDetection:public FaultDetection{
	public:
		ROSFaultDetection(ros::NodeHandle nh) : FaultDetection(true){
		}

		Mat setFrame(){
			return Mat();
		}
		void startDetection(){
		  start();
		  while(ros::ok()){
				mtx_.lock(); // not proceed until new frame is received
		    run();
		  }

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
			mtx_.unlock();
			//frame_ = cv_ptr->image;
	 }
	private:
		std::mutex mtx_;
};

#endif /* ROSFAULTDETECTION_H_ */
