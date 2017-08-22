#include "vision_utils_ros/ros_fault_detection.h"

ROSFaultDetection::ROSFaultDetection(ros::NodeHandle nh, int hessian) : current_(), last_(), cusum_(0.0), is_First_Image_received(false),detector_(hessian){
  ROS_INFO("ROSFaultDetection Constructor");
  image_sub_ = nh.subscribe("/cam3d/rgb/image_raw", 1, &ROSFaultDetection::imageCb,this);
  image_pub_ = nh.advertise<sensor_msgs::Image>("Image", 1);
  cusum_pub_ = nh.advertise<std_msgs::Float64>("cusum_surf_distance", 1);
  ros::NodeHandle nh2("~");
  output_msg_pub_ = nh2.advertise<fusion_msgs::sensorFusionMsg>("/collisions_2", 1);
  ros::spin();
}

ROSFaultDetection::~ROSFaultDetection(){

}

void ROSFaultDetection::imageCb(const sensor_msgs::ImageConstPtr& msg){
  cv_bridge::CvImagePtr cv_ptr;
  Mat input_frame;

  try
   {
    //cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
    input_frame = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
    current_.setFrame(input_frame);

   }
   catch (cv_bridge::Exception& e)
   {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
   }

  if (!current_.getFrame().empty()){
    runFeatureExtractor();

  }
  if (!is_First_Image_received){
    is_First_Image_received = true;
    ROS_INFO("First Frame Received");
    last_ = current_;

  }
  else{
    run();
    last_ = current_;
  }
}

void ROSFaultDetection::run(){
   matcher_.clearing();
   matcher_.matchD(current_,last_);
   matcher_.separateMatches(current_,last_);
   matcher_.getBestMatches(current_,last_);
   matcher_.separateBestMatches(current_,last_);
   matcher_.drawBestMatches(current_,last_);
   cusum_ = statics_tool->CUSUM(matcher_);
   publishOutputs();
}

void ROSFaultDetection::publishOutputs(){

 // Publishing image after matching
 Mat img = matcher_.getFrame();
 sensor_msgs::Image out_msg;
 cv_bridge::CvImage img_bridge;
 std_msgs::Header header;

 try{
   img_bridge = cv_bridge::CvImage(header, sensor_msgs::image_encodings::RGB8, img);
   img_bridge.toImageMsg(out_msg); // from cv_bridge to sensor_msgs::Image
   image_pub_.publish(out_msg);
 }

 catch (cv_bridge::Exception& e){
   ROS_ERROR("cv_bridge exception: %s", e.what());
   return;
 }

 //CUSUM Plot
 std_msgs::Float64 out_msg_2;
 out_msg_2.data = cusum_;
 cusum_pub_.publish(out_msg_2);

 //sensorFusionMsg
 std_msgs::Float32 msg;
 //msg.data = cusum_;
 output_msg_pub_.publish(output_msg_);
}

void ROSFaultDetection::runFeatureExtractor(){

 Mat img,descriptors;// = first_.getDescriptors();
 img = current_.getFrame();
 std::vector<cv::KeyPoint> k1;// = first_.getKeyPoints();
 Mat tmp;

 detector_.detect( img, k1 );
 drawKeypoints(img,k1,tmp);
 extractor_.compute(img, k1, descriptors);

 if (descriptors.type()!=CV_32F) {
   descriptors.convertTo(descriptors, CV_32F);
 }

 if (!tmp.empty()) {
   current_.setFrame(tmp);
   current_.setDescriptors(descriptors);
   current_.setKeyPoints(k1);
 }

}
