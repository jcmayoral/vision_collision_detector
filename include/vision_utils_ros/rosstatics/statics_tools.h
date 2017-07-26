#include<opencv2/imgproc.hpp>
//#include <opencv2/plot.hpp>
#include <vision_utils_ros/rosfeaturedetection/matcher.h>
#include <math.h>

#ifndef STATICS_TOOLS_H
#define STATICS_TOOLS_H
class MyStatics{
  public:
    MyStatics();
    ~MyStatics();
    Point getKMeans(const Matcher input);
    Point calculateMean(Matcher match);
    Point calculateVariance(Matcher match, Point mean);
    double CalculateCovariance(Matcher match , double meanx, double meany);
    double CalculatePearsonCorrelation(Matcher match , double meanx, double meany, double varx, double vary);
    double CUSUM(Matcher input);
};
#endif // STATICS_TOOLS_H
