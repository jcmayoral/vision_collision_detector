/*
 * tracker.cpp
 *
 *  Created on: Nov 29, 2016
 *      Author: jose
 */

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "vision_utils_ros/rosfeaturedetection/tracker.h"
#include <iostream>
#include <iterator>

Tracker::Tracker(){
};

Tracker::~Tracker(){

};

void Tracker::featureTracking(MyFrameContainer f1, MyFrameContainer f2, Matcher& match)	{

//this function automatically gets rid of points for which tracking fails
  std::vector<float> err;
  std::vector<uchar> status;
  Size winSize=Size(21,21);
  TermCriteria termcrit=TermCriteria(TermCriteria::COUNT+TermCriteria::EPS, 30, 0.01);

  calcOpticalFlowPyrLK(f1.getFrame(), f2.getFrame(), match.getVector(3) , match.getVector(2), status, err, winSize, 3, termcrit, 0, 0.001);

  std::vector<Point2f> v = match.getVector(2);
  std::vector<Point2f> v2 = match.getVector(3);
  //getting rid of points for whic  h the KLT tracking failed or those who have gone outside the frame
  int indexCorrection = 0;
  for( int i=0; i<status.size(); i++)
     {  Point2f pt = v.at(i- indexCorrection);
        if ((status.at(i) == 0)||(pt.x<0)||(pt.y<0))	{
              if((pt.x<0)||(pt.y<0))	{
                status.at(i) = 0;
              }
              v2.erase (v2.begin() + (i - indexCorrection));
              v.erase (v.begin() + (i - indexCorrection));
              indexCorrection++;
        }

     }
}
