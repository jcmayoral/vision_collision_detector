#include "vision_utils_ros/ros_matcher.h"

ROSMatcher::ROSMatcher(double threshold): frame_(), matchPercentage_(threshold){
  matchPercentage_ = threshold;
}

ROSMatcher::~ROSMatcher(){

}

void ROSMatcher::setMatchPercentage(double val){
  matchPercentage_ = val;
}

double ROSMatcher::getMatchPercentage(){
  return matchPercentage_;
}

void ROSMatcher::matchD(MyFrameContainer ext1, MyFrameContainer ext2){
  match(ext1.getDescriptors(),ext2.getDescriptors(),matches_);
}

void ROSMatcher::separateMatches(MyFrameContainer ext1, MyFrameContainer ext2){
  std::vector<cv::KeyPoint> k1 = ext1.getKeyPoints();
  std::vector<cv::KeyPoint> k2 = ext2.getKeyPoints();

  for (unsigned int i= 0; i< matches_.size();i++){
    query_.push_back(k1[matches_[i].queryIdx].pt);
		train_.push_back(k2[matches_[i].queryIdx].pt);
		//std::cout << matches_[i].distance << std::endl;
  }
}


void ROSMatcher::separateBestMatches(MyFrameContainer ext1, MyFrameContainer ext2){
  std::vector<cv::KeyPoint> k1 = ext1.getKeyPoints();
  std::vector<cv::KeyPoint> k2 = ext2.getKeyPoints();

	for (unsigned int i= 0; i< best_matches_.size();i++){
    best_query_.push_back(k1[best_matches_[i].queryIdx].pt);
		best_train_.push_back(k2[best_matches_[i].queryIdx].pt);
		//std::cout << matches_[i].distance << std::endl;
	}
}

void ROSMatcher::getBestMatches(MyFrameContainer ext1, MyFrameContainer ext2){
    /*From \cite{Mori2013} First Results in Deeting and Avoiding Frontal Obstacles from
    * A Monocular Camera fro Micro Unmanned Aerial Vehicles
    */
    std::vector<cv::KeyPoint> k1 = ext1.getKeyPoints();
    std::vector<cv::KeyPoint> k2 = ext2.getKeyPoints();
    Mat descriptors = ext1.getDescriptors();

    for( int i = 0; i < descriptors.rows; i++ ){
      if ((k2[i].size > k1[i].size)&& (matches_[i].distance< matchPercentage_)){
        best_matches_.push_back( matches_[i]);
		}
	}
}

void ROSMatcher::drawBestMatches(MyFrameContainer ext1, MyFrameContainer ext2){
  drawMatches(ext1.getFrame(), ext1.getKeyPoints(), ext2.getFrame(), ext2.getKeyPoints(), best_matches_,frame_, Scalar::all(-1), Scalar::all(-1),std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
}

void ROSMatcher::drawAllMatches(MyFrameContainer ext1, MyFrameContainer ext2){
	drawMatches(ext1.getFrame(), ext1.getKeyPoints(), ext2.getFrame(), ext2.getKeyPoints(), matches_,frame_, Scalar::all(-1), Scalar::all(-1),std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
}

void ROSMatcher::show(std::string window_name){
  if (!frame_.empty()) {
      imshow(window_name, frame_);
  }
}

void ROSMatcher::clearing(){
  query_.clear();
	train_.clear();
	best_query_.clear();
	best_train_.clear();
	matches_.clear();
	best_matches_.clear();
}

int ROSMatcher::getSize(int index){
  switch (index){
    case 0: return query_.size();
    case 1: return train_.size();
    case 2: return best_query_.size();
    case 3: return best_train_.size();
    case 4: return best_matches_.size();
    default: return 0;
  }
}

std::vector<Point2f> ROSMatcher::getVector(int index){
  switch (index){
    case 0: return query_;
    case 1: return train_;
    case 2: return best_query_;
    case 3: return best_train_;
  }
}

std::vector<DMatch> ROSMatcher::getBestMatches(){
  return best_matches_;
}

void ROSMatcher::setFrame(Mat new_frame){
  frame_ = new_frame;
}

Mat ROSMatcher::getFrame(){
  return frame_;
}
