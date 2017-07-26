#include "vision_utils_ros/rosfeaturedetection/matcher.h"

Matcher::Matcher(): frame_(), matchPercentage_(0.10){
  std::cout << "ROS Matcher" << std::endl;;
}

Matcher::~Matcher(){

}

void Matcher::setMatchPercentage(double val){
  matchPercentage_ = val;
}

void Matcher::matchD(MyFrameContainer ext1, MyFrameContainer ext2){
  match(ext1.getDescriptors(),ext2.getDescriptors(),matches_);
}

void Matcher::separateMatches(MyFrameContainer ext1, MyFrameContainer ext2){
  std::vector<cv::KeyPoint> k1 = ext1.getKeyPoints();
  std::vector<cv::KeyPoint> k2 = ext2.getKeyPoints();

  for (unsigned int i= 0; i< matches_.size();i++){
    query_.push_back(k1[matches_[i].queryIdx].pt);
		train_.push_back(k2[matches_[i].queryIdx].pt);
		//std::cout << matches_[i].distance << std::endl;
  }
}


void Matcher::separateBestMatches(MyFrameContainer ext1, MyFrameContainer ext2){
  std::vector<cv::KeyPoint> k1 = ext1.getKeyPoints();
  std::vector<cv::KeyPoint> k2 = ext2.getKeyPoints();

	for (unsigned int i= 0; i< best_matches_.size();i++){
    best_query_.push_back(k1[best_matches_[i].queryIdx].pt);
		best_train_.push_back(k2[best_matches_[i].queryIdx].pt);
		//std::cout << matches_[i].distance << std::endl;
	}
}

void Matcher::getBestMatches(MyFrameContainer ext1, MyFrameContainer ext2){
    /*From \cite{Mori2013} First Results in Deeting and Avoiding Frontal Obstacles from
    * A Monocular Camera fro Micro Unmanned Aerial Vehicles
    */
    std::vector<cv::KeyPoint> k1 = ext1.getKeyPoints();
    std::vector<cv::KeyPoint> k2 = ext2.getKeyPoints();
    Mat descriptors = ext1.getDescriptors();

    for( int i = 0; i < descriptors.rows; i++ ){
      if ((k2[i].size > k1[i].size)&& (matches_[i].distance<0.03)){
        best_matches_.push_back( matches_[i]);
		}
	}
}

void Matcher::drawBestMatches(MyFrameContainer ext1, MyFrameContainer ext2){
  drawMatches(ext1.getFrame(), ext1.getKeyPoints(), ext2.getFrame(), ext2.getKeyPoints(), best_matches_,frame_, Scalar::all(-1), Scalar::all(-1),std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
}

void Matcher::drawAllMatches(MyFrameContainer ext1, MyFrameContainer ext2){
	drawMatches(ext1.getFrame(), ext1.getKeyPoints(), ext2.getFrame(), ext2.getKeyPoints(), matches_,frame_, Scalar::all(-1), Scalar::all(-1),std::vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
}

void Matcher::show(std::string window_name){
  imshow(window_name, frame_);
}

void Matcher::clearing(){
  query_.clear();
	train_.clear();
	best_query_.clear();
	best_train_.clear();
	matches_.clear();
	best_matches_.clear();
}

int Matcher::getSize(int index){
  switch (index){
    case 0: return query_.size();
    case 1: return train_.size();
    case 2: return best_query_.size();
    case 3: return best_train_.size();
    case 4: return best_matches_.size();
    default: return 0;
  }
}

std::vector<Point2f> Matcher::getVector(int index){
  switch (index){
    case 0: return query_;
    case 1: return train_;
    case 2: return best_query_;
    case 3: return best_train_;
  }
}

std::vector<DMatch> Matcher::getBestMatches(){
  return best_matches_;
}
