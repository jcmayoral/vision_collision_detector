#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <featuredetection/FD.h>

using namespace cv;
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

		virtual bool start() {
			ROS_INFO("This one");
		}


		virtual bool run() {
			ROS_INFO("This one");
			first_.read(camera_);
		  matcher_.clearing();
		  first_.ORB();
		  return true;
		}

		virtual bool stop() {
			ROS_INFO("This one");
		}
		/*
		//Override Methods
		void Matcher::show(std::string window_name){
			cout << "New Show";
		}
*/
		void startDetection(){
			ROS_INFO("startDetection");
		  start();
			ROS_INFO("runDetection");
			//cv::imshow("hola", first_.frame_);
			run();
		  stop();
		}

		void imageCb(const sensor_msgs::ImageConstPtr& msg){
			//mtx_.lock();
			cv_bridge::CvImagePtr cv_ptr;

			try{
				cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
			}
			catch (cv_bridge::Exception& e){
				ROS_ERROR("cv_bridge exception: %s", e.what());
				return;
		  }
			first_.setFrame(cv_ptr->image);

			cv_bridge::CvImage out_msg;
			//out_msg.header   = in_msg->header; // Same timestamp and tf frame as input image
			out_msg.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
			out_msg.image    = first_.frame_; // Your cv::Mat
			image_pub_.publish(out_msg);


			if (is_First_Image_received){
				//startDetection();
				is_First_Image_received = false;
			}
			else{
				first_.ORB();
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
