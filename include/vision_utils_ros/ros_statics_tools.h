//#include<opencv2/imgproc.hpp>
//#include <opencv2/plot.hpp>
#include "vision_utils_ros/ros_matcher.h"
#include <math.h>

#ifndef STATICS_TOOLS_H
#define STATICS_TOOLS_H
class ROSStatics{
  public:
    ROSStatics();
    ~ROSStatics();
    Point calculateMean(ROSMatcher match);
    Point calculateVariance(ROSMatcher match, Point mean);
    double CalculateCovariance(ROSMatcher match , double meanx, double meany);
    double CalculatePearsonCorrelation(ROSMatcher match , double meanx, double meany, double varx, double vary);
    double CUSUM(ROSMatcher input);
};
#endif // STATICS_TOOLS_H
