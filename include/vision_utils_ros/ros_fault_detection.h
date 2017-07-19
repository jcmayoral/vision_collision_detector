#include <ros/ros.h>
#include <featuredetection/FD.h>
using namespace cv;
using namespace std;

#ifndef ROSFAULTDETECTION_H_
#define ROSFAULTDETECTION_H_

//Override Methods
void Matcher::show(std::string window_name){
	cout << "New Show";
}

void MyFeatureExtractor::read(VideoCapture v){
	cout << "my new read" << endl;
}

bool FaultDetection::run(){
  return true;
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
		void startDetection(){
		  start();
		  while(ros::ok()){
		    run();
		  }
		  stop();
		}
};

#endif /* ROSFAULTDETECTION_H_ */
