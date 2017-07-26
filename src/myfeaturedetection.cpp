/*
 * myfeacturedetection.cpp
 *
 *  Created on: Nov 24, 2016
 *      Author: jose
 */

#include "vision_utils_ros/rosfeaturedetection/myfeaturedetection.h"
#include "opencv2/videoio.hpp"

MyFrameContainer::MyFrameContainer(){
    std::cout << "ROS MyFrameContainer Constructor" << std::endl;
}

MyFrameContainer::MyFrameContainer(const MyFrameContainer& other): descriptors_(other.descriptors_),
                                            keypoints_(other.keypoints_), frame_(other.frame_){
    //cv::cvtColor(frame_,frame_,cv::COLOR_RGB2GRAY);
}

MyFrameContainer::~MyFrameContainer(){
    frame_.release();
}

void MyFrameContainer::show(std::string window_name){
	imshow(window_name, frame_);
}

void MyFrameContainer::setFrame(Mat new_frame){
  frame_ = new_frame;
}

Mat MyFrameContainer::getFrame(){
  return frame_;
}

void MyFrameContainer::setDescriptors(Mat desc){
  descriptors_ = desc;
}

Mat MyFrameContainer::getDescriptors(){
  return descriptors_;
}

std::vector<cv::KeyPoint> MyFrameContainer::getKeyPoints(){
  return keypoints_;
}

void MyFrameContainer::setKeyPoints(std::vector<cv::KeyPoint> kp){
  keypoints_ = kp;
}

void MyFrameContainer::read(VideoCapture v){
    v  >> frame_;
    //cv::cvtColor(frame_,frame_,cv::COLOR_BGR2GRAY);
}
