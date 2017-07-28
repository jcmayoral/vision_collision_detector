//ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
//OpenCV
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include <opencv2/imgproc/imgproc.hpp>
//VisionUtils
#include <vision_utils_ros/ros_statics_tools.h>
//vision_utils_ros
#include <vision_utils_ros/frame_container.h>
#include <vision_utils_ros/ros_matcher.h>

using namespace cv;
using namespace std;

#ifndef ROSFAULTDETECTION_H_
#define ROSFAULTDETECTION_H_

class ROSFaultDetection{
	public:

		ROSFaultDetection(ros::NodeHandle nh, int hessian=400);
		~ROSFaultDetection();
		void imageCb(const sensor_msgs::ImageConstPtr& msg);
		void run();
		void publishOutputs();
		void runFeatureExtractor();

	private:
		//Contains information from a given frame
		MyFrameContainer current_;
		MyFrameContainer last_;
		//SURF Extraction
		SurfFeatureDetector detector_;
		SurfDescriptorExtractor extractor_;
		ROSMatcher matcher_;

		//Flags
		bool is_First_Image_received;

		//Publishers and Subscriber
		ros::Subscriber image_sub_;
		ros::Publisher image_pub_;
		ros::Publisher cusum_pub_;

		//CUUSM
		double cusum_;
		std::shared_ptr<ROSStatics> statics_tool;
};

#endif /* ROSFAULTDETECTION_H_ */
