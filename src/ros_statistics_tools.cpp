#include <vision_utils_ros/ros_statistics_tools.h>
#include<random>

using namespace std;
using namespace cv;

ROSStatistics::ROSStatistics(){
}

ROSStatistics::~ROSStatistics(){
}

Point ROSStatistics::calculateMean(ROSMatcher match){

    int number_points = match.getSize(2);
    std::vector<Point2f> v = match.getVector(2);
    Point tmp;
    tmp.x = 0;
    tmp.y = 0;

    for (unsigned int i=0; i<match.getSize(2);i++){
        tmp.x = tmp.x + (v.at(i).x);
        tmp.y = tmp.y + (v.at(i).y);
    }

    if (number_points > 0){
        tmp.x /= number_points;
        tmp.y /= number_points;
    }

    return tmp;
}

Point ROSStatistics::calculateVariance(ROSMatcher match, Point mean){
    int number_points = match.getSize(2);
    std::vector<Point2f> v = match.getVector(2);
    Point tmp;
    tmp.x = 0;
    tmp.y = 0;
    for (unsigned int i=0; i<match.getSize(2);i++){
        tmp.x += std::pow(v.at(i).x - mean.x,2);
        tmp.y += std::pow(v.at(i).y - mean.y,2);
    }

    if (number_points > 1){
        tmp.x /= (number_points-1);
        tmp.y /= (number_points-1);
    }

    else{
        tmp.x =0.0;
        tmp.y = 0.0;
    }

    return tmp;
}

double ROSStatistics::CalculateCovariance(ROSMatcher match , double meanx, double meany){

    int number_points = match.getSize(2);
    std::vector<Point2f> v = match.getVector(2);
    double tmp=0.0;

    if (number_points == 0){
        return 0.0;
    }

    for (unsigned int i=0; i<match.getSize(2);i++){
        tmp += (v[i].x - meanx) * (v[i].y -meany);
    }

    if(number_points>0){
        tmp/=number_points;
    }
    else{
        tmp = 0.0;
    }
    return tmp;
}

double ROSStatistics::CalculatePearsonCorrelation(ROSMatcher match , double meanx, double meany, double varx, double vary){

    int number_points = match.getSize(2);
    std::vector<Point2f> v = match.getVector(2);
    double tmp=0.0;

    if (number_points == 0){
        return 0.0;
    }

    for (unsigned int i=0; i<match.getSize(2);i++){
        tmp += (v[i].x - meanx) * (v[i].y -meany);
    }

    if((varx*vary)>1e-20){
        tmp/=(varx*vary);
    }
    else{
      return 0.0;
    }

    return tmp;
}

double ROSStatistics::CUSUM(ROSMatcher input, double & last_mean, double & last_variance, double last_cusum){

  double std_deviation, last_std_deviation = 1.0;
  double cusum_mean = last_mean;
  double cusum_var = last_variance;
  double cusum = last_cusum;

  std::vector<DMatch> v = input.getBestMatches();
  double max_value = input.getMatchPercentage();

  if (input.getSize(4)>2){
    //mean
    cusum_mean = 0.0;
    cusum_var = 0.0;
    cusum = 0.0;

    for (unsigned int i=0; i<input.getSize(4);i++){
      cusum_mean += v[i].distance/max_value;
    }
    cusum_mean = cusum_mean/input.getSize(4);

    //variance
    for (unsigned int i=0; i<input.getSize(4);i++){
      cusum_var += std::pow(v[i].distance - cusum_mean,2);
    }
    cusum_var = cusum_var/(input.getSize(4)-1);
    //ROS_INFO_STREAM("size" << input.getSize(4)-1);
    //ROS_INFO_STREAM("variance " << cusum_var);

    // BLANKE
    // s_z = (-np.power(z-m1,2) + np.power(z-m0,2))/(2*v1)
    std_deviation = sqrt(cusum_var);
    //ROS_INFO_STREAM("std_deviation " << std_deviation);
    last_std_deviation = sqrt(last_variance);

    double constant = ((1/last_std_deviation) - (1/std_deviation))/2;
    //ROS_INFO_STREAM("constant " << constant);

    double prefix_constant = log(last_std_deviation/std_deviation);
    //ROS_INFO_STREAM("prefix_constant " << prefix_constant);

    for (unsigned int i=0; i<input.getSize(4);i++){
      cusum += prefix_constant + constant * pow((v[i].distance/max_value)-cusum_mean,2);
      last_mean = cusum_mean;
      last_variance = cusum_var;
    }
  }


  return fabs(cusum);
}
