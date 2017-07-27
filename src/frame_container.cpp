/*
 * frame_container.cpp
 *
 *  Created on: Jul 27, 2017
 *      Author: jose
 */

#include "vision_utils_ros/frame_container.h"

MyFrameContainer::MyFrameContainer(){
}

MyFrameContainer::~MyFrameContainer(){
}

MyFrameContainer::MyFrameContainer(const MyFrameContainer& other): descriptors_(other.descriptors_),
                                            keypoints_(other.keypoints_), frame_(other.frame_){
    //cv::cvtColor(frame_,frame_,cv::COLOR_RGB2GRAY);
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

/*
std::vector<cv::KeyPoint> FrameContainer::getKeyPoints(){
  return keypoints_;
}

void FrameContainer::setKeyPoints(std::vector<cv::KeyPoint> kp){
  keypoints_ = kp;
}
void FrameContainer::read(VideoCapture v){
    v  >> frame_;
    //cv::cvtColor(frame_,frame_,cv::COLOR_BGR2GRAY);
}

*/
