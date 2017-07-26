/*
 * MVO.h
 *
 *  Created on: Nov 24, 2016
 *      Author: jose
 */

//#include "../monocularvision/featuredetection.h"
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <vision_utils_ros/rosfeaturedetection/matcher.h>
#include <vision_utils_ros/rosfeaturedetection/tracker.h>
#include <vision_utils_ros/rosstatics/statics_tools.h>
#include <memory>

using namespace cv;
using namespace cv::xfeatures2d;

class ROSFaultDetection{
public:
  ROSFaultDetection(ros::NodeHandle nh);
  ROSFaultDetection(bool ros);
  ~ROSFaultDetection();
  bool start();
  bool run();
  bool stop();
  void runSURF();
  Matcher getMatcher();
  Point getMeanPoint();
  Point getVariance();
  Point getCurrentCenter();
  double getCovariance();
  double getPearson();
  double getCUSUM();
  void imageCb(const sensor_msgs::ImageConstPtr& msg);

protected:
  //MyFrameContainer first_;
  //MyFrameContainer second_;
  //Ptr<SURF> fDetector_;
  Matcher matcher_;
  Point currentMeanPoint_;
  Point currentVariancePoint_;
  Point currentCenter_;
  double pearson_;
  double covariance_;
  double cusum_;
  VideoCapture camera_;
  MyStatics statics_tool;

  bool is_First_Image_received;
  ros::Subscriber image_sub_;
  ros::Publisher image_pub_;

};
