#include <vision_utils_ros/ros_statics_tools.h>
#include<random>

using namespace std;
using namespace cv;

ROSStatics::ROSStatics(){
}

ROSStatics::~ROSStatics(){
}

Point ROSStatics::calculateMean(ROSMatcher match){

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

Point ROSStatics::calculateVariance(ROSMatcher match, Point mean){
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

double ROSStatics::CalculateCovariance(ROSMatcher match , double meanx, double meany){

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

double ROSStatics::CalculatePearsonCorrelation(ROSMatcher match , double meanx, double meany, double varx, double vary){

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

double ROSStatics::CUSUM(ROSMatcher input){

    double cusum_mean, cusum_var = 0.0;
    std::vector<DMatch> v = input.getBestMatches();

    if (input.getSize(4)>1){
      for (unsigned int i=0; i<input.getSize(4);i++){
        cusum_mean += v[i].distance;
      }

      cusum_mean = cusum_mean/input.getSize(4);

      for (unsigned int i=0; i<input.getSize(4);i++){
          cusum_var += std::pow(v[i].distance - cusum_mean,2);
      }

      cusum_var = cusum_var/(input.getSize(4)-1);
    }

    return cusum_var;
}
