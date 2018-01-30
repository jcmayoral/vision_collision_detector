/*
 * framecontainer.h
 *
 *  Created on: Nov 24, 2016
 *      Author: jose
 */

/*
#include "opencv2/opencv.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/opencv.hpp"
using namespace cv;
*/
#include "opencv2/highgui/highgui.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d/features2d.hpp>

using namespace cv;

#ifndef FRAMECONTAINER_H_
#define FRAMECONTAINER_H_

class MyFrameContainer{
	public:
		MyFrameContainer();
    MyFrameContainer(const MyFrameContainer& other);
		~MyFrameContainer();
		void setFrame(Mat new_frame);
		Mat getFrame();
		void setDescriptors(Mat desc);
		Mat getDescriptors();
		std::vector<cv::KeyPoint> getKeyPoints();
		void setKeyPoints(std::vector<cv::KeyPoint> kp);
		void clearFrame();

	private:
		Mat frame_;
		Mat descriptors_;
		std::vector<cv::KeyPoint> keypoints_;

};

#endif /* FRAMECONTAINER_H_ */
