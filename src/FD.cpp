/*
 * MVO.cpp
 *
 *  Created on: Nov 24, 2016
 *      Author: jose
 */

#include "opencv2/opencv.hpp"
#include <iostream>
#include <exception>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <rosfeaturedetection/FD.h>
#include <rosstatics/statics_tools.h>

using namespace cv;
using namespace std;

ROSFaultDetection::ROSFaultDetection(ros::NodeHandle nh): is_First_Image_received(false){
  // TODO Auto-generated constructor stub
  ROS_INFO("aaaaaaaaaaaaaaaaaaaaaaaaaaa puta madre");
    image_pub_ =nh.advertise<sensor_msgs::Image>("image_detection", 1);
    image_sub_ = nh.subscribe("/camera/rgb/image_color", 1, &ROSFaultDetection::imageCb,this);
    ROS_INFO("Before surf");
    /*
    fDetector_->setHessianThreshold(400);
    fDetector_ = SURF::create();
    matcher_.setMatchPercentage(0.05);
    */
}

ROSFaultDetection::~ROSFaultDetection() {
	// TODO Auto-generated destructor stub
  //delete this->statics_tool;
  cout << "FD destroyed";
}

Matcher ROSFaultDetection::getMatcher(){
  return matcher_;
}

Point ROSFaultDetection::getMeanPoint(){
  return currentMeanPoint_;
}

Point ROSFaultDetection::getVariance(){
  return currentVariancePoint_;
}

Point ROSFaultDetection::getCurrentCenter(){
  return currentCenter_;
}

double ROSFaultDetection::getCovariance(){
  return covariance_;
}

double ROSFaultDetection::getPearson(){
  return pearson_;
}

double ROSFaultDetection::getCUSUM(){
    return cusum_;
}

bool ROSFaultDetection::start(){
  if (!camera_.isOpened()){
    cerr  << "Could not open the input video: " << endl;
    return false;
  }

  //first_.read(camera_);
  runSURF();
  return true;
}

bool ROSFaultDetection::run(){
  Tracker tracker;
  /*
  try{
    //second_ = first_;
    //first_.read(camera_);
    matcher_.clearing();
    runSURF();
    matcher_.matchD(first_,second_);
    matcher_.separateMatches(first_,second_);
    matcher_.getBestMatches(first_,second_);
    matcher_.separateBestMatches(first_,second_);
    //tracker.featureTracking(first_, second_,matcher_);
    matcher_.drawBestMatches(first_,second_);
    matcher_.show("BestMatchesDisplay");
    currentMeanPoint_ = statics_tool.calculateMean(matcher_);
    //currentCenter_ = statics_tool->getKMeans(matcher_);
    currentVariancePoint_ = statics_tool.calculateVariance(matcher_,currentMeanPoint_);
    covariance_ = statics_tool.CalculateCovariance(matcher_,currentMeanPoint_.x,currentMeanPoint_.y);
    pearson_ = statics_tool.CalculatePearsonCorrelation(matcher_,currentMeanPoint_.x,currentMeanPoint_.y, currentVariancePoint_.x, currentVariancePoint_.y);
    cusum_ = statics_tool.CUSUM(matcher_);
    return true;
  }
  catch(Exception e){
    e.what();
    return false;
  }
  */
}

bool ROSFaultDetection::stop(){
    camera_.release();
    matcher_.clearing();
    destroyAllWindows();
    cout << "MVO finishing correctly" << endl;
    return true;
}

void ROSFaultDetection::imageCb(const sensor_msgs::ImageConstPtr& msg){
  //mtx_.lock();
  cv_bridge::CvImagePtr cv_ptr;
  cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  //first_.setFrame(cv_ptr->image);
  runSURF();
  cv_bridge::CvImage out_msg;
  //out_msg.header   = in_msg->header; // Same timestamp and tf frame as input image
  out_msg.encoding = sensor_msgs::image_encodings::RGB8; // Or whatever
  out_msg.image    = cv_ptr->image; // Your cv::Mat
  image_pub_.publish(out_msg);

  if (is_First_Image_received){
    //startDetection();
    is_First_Image_received = false;
  }
  else{
    //first_.ORB();
    //second_ = first_;
    ROS_INFO("Copy first to second");
    is_First_Image_received = true;
  }
}

void ROSFaultDetection::runSURF(){
  Mat img,descriptors;// = first_.getDescriptors();
  std::cout << "MatGetter";
  //img = first_.getFrame();
  std::cout << "Key Point";
  std::vector<cv::KeyPoint> k1;// = first_.getKeyPoints();
  //fDetector_->detect(first_.getFramei(),k1);
  //fDetector_->compute(first_.getFrame(), k1, descriptors);
  Mat tmp;
  //fDetector_->detectAndCompute(img,tmp, k1, descriptors);
  //drawKeypoints(img,k1,tmp);


  if (descriptors.type()!=CV_32F) {
    descriptors.convertTo(descriptors, CV_32F);
  }

  //first_.setFrame(tmp);
  //first_.setDescriptors(descriptors);
  //first_.setKeyPoints(k1);
}
