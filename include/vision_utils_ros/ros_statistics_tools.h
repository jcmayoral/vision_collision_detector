//#include<opencv2/imgproc.hpp>
//#include <opencv2/plot.hpp>
#include "vision_utils_ros/ros_matcher.h"
#include <ros/ros.h>

#include <math.h>

#ifndef STATISTICS_TOOLS_H
#define STATISTICS_TOOLS_H
class ROSStatistics{
  public:
    ROSStatistics();
    ~ROSStatistics();
    Point calculateMean(ROSMatcher match);
    Point calculateVariance(ROSMatcher match, Point mean);
    double CalculateCovariance(ROSMatcher match , double meanx, double meany);
    double CalculatePearsonCorrelation(ROSMatcher match , double meanx, double meany, double varx, double vary);
    double CUSUM(ROSMatcher input, double & last_mean, double & last_variance, double last_cusum);
    double getBlur(Mat currentFrame);
};
#endif // STATICS_TOOLS_H
