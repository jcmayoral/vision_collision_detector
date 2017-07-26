#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <vision_utils_ros/rosfeaturedetection/myfeaturedetection.h>
#include <vision_utils_ros/rosfeaturedetection/matcher.h>
using namespace cv;

class Tracker{

	public:
		Tracker();
		~Tracker();
		void featureTracking(MyFrameContainer f1, MyFrameContainer f2, Matcher& match);
};
