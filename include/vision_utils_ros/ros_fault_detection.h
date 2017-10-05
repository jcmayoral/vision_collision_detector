//ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

//dynamic_reconfigure
#include <dynamic_reconfigure/server.h>
#include <vision_utils_ros/dynamic_reconfigureConfig.h>

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

#include <fusion_msgs/sensorFusionMsg.h>

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
    void dyn_reconfigureCB(vision_utils_ros::dynamic_reconfigureConfig &config, uint32_t level);


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
		ros::Publisher output_msg_pub_;

		//CUUSM
		double cusum_;
		double last_cusum_;
		std::shared_ptr<ROSStatics> statics_tool;

		//Output
		double collisions_threshold_;
		fusion_msgs::sensorFusionMsg output_msg_;

		//Strings
		std::string sensor_id_;
		std::string frame_id_;

		//dynamic_reconfigure
		dynamic_reconfigure::Server<vision_utils_ros::dynamic_reconfigureConfig> dyn_server;
    dynamic_reconfigure::Server<vision_utils_ros::dynamic_reconfigureConfig>::CallbackType dyn_server_cb;

};

#endif /* ROSFAULTDETECTION_H_ */
