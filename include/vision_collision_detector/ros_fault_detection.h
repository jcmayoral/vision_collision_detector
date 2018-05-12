//ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float32.h>

//dynamic_reconfigure
#include <dynamic_reconfigure/server.h>
#include <vision_collision_detector/dynamic_reconfigureConfig.h>

//OpenCV
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"

#include <opencv2/imgproc/imgproc.hpp>
//vision_collision_detector
#include <vision_collision_detector/ros_statistics_tools.h>
#include <vision_collision_detector/frame_container.h>
#include <vision_collision_detector/ros_matcher.h>

#include <fusion_msgs/sensorFusionMsg.h>

using namespace cv;
using namespace std;
using namespace cv::xfeatures2d;

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
		double detectBlur();
    void dyn_reconfigureCB(vision_collision_detector::dynamic_reconfigureConfig &config, uint32_t level);
		void reset();


	private:
		//Contains information from a given frame
		MyFrameContainer current_;
		MyFrameContainer last_;
		//SURF Extraction
		Ptr<SURF> detector_;
		ROSMatcher matcher_;

		//Flags
		bool is_First_Image_received;
		bool is_disabled_;
		//Publishers and Subscriber
		ros::Subscriber image_sub_;
		ros::Publisher image_pub_;
		ros::Publisher cusum_pub_;
		ros::Publisher output_msg_pub_;

		//CUUSM55
		double cusum_;
		double last_cusum_;
		double last_cusum_mean_;
		double last_cusum_var_;
		std::shared_ptr<ROSStatistics> statics_tool;

		//Output
		double collisions_threshold_;
		fusion_msgs::sensorFusionMsg output_msg_;

		//Strings
		std::string sensor_id_;
		std::string frame_id_;

		//Mode Selector
		int mode_;

		//Weight
		double weight_;

		//dynamic_reconfigure
		dynamic_reconfigure::Server<vision_collision_detector::dynamic_reconfigureConfig> dyn_server;
    dynamic_reconfigure::Server<vision_collision_detector::dynamic_reconfigureConfig>::CallbackType dyn_server_cb;

};

#endif /* ROSFAULTDETECTION_H_ */
