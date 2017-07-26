#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vision_utils_ros/rosfeaturedetection/myfeaturedetection.h>
using namespace cv;


class Matcher:FlannBasedMatcher{

	public:
		Matcher();
		~Matcher();
		void show(std::string window_name);
		void setMatchPercentage(double val);
		void getBestMatches(MyFrameContainer ext1, MyFrameContainer ext2);
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
