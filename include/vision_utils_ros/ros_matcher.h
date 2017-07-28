#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "frame_container.h"
using namespace cv;

#ifndef ROSMATCHER_H_
#define ROSMATCHER_H_

class ROSMatcher:FlannBasedMatcher{

	public:
		ROSMatcher();
		~ROSMatcher();
		void show(std::string window_name);
		void setMatchPercentage(double val);
		void getBestMatches(MyFrameContainer ext1, MyFrameContainer ext2);
		void setFrame(Mat new_frame);
		Mat getFrame();
		void matchD(MyFrameContainer ext1, MyFrameContainer ext2);
		void separateMatches(MyFrameContainer ext1, MyFrameContainer ext2);
		void separateBestMatches(MyFrameContainer ext1, MyFrameContainer ext2);
		void drawBestMatches(MyFrameContainer ext1, MyFrameContainer ext2);
		void drawAllMatches(MyFrameContainer ext1, MyFrameContainer ext2);
		void clearing();
		int getSize(int index);
		std::vector<Point2f> getVector(int index);
		std::vector<DMatch> getBestMatches();

	protected:
		std::vector<Point2f> query_;
		std::vector<Point2f> train_;
		std::vector<Point2f> best_query_;
		std::vector<Point2f> best_train_;
		std::vector<DMatch> matches_;
		std::vector<DMatch> best_matches_;
		Mat frame_;
		double matchPercentage_;

};

#endif /* ROSMatcher_H_ */
