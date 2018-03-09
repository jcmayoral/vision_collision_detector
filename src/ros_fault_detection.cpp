#include "vision_utils_ros/ros_fault_detection.h"

ROSFaultDetection::ROSFaultDetection(ros::NodeHandle nh, int hessian) : current_(), last_(), last_cusum_(0.0), cusum_(0.0), last_cusum_mean_(0.0),
                                                                        last_cusum_var_(1.0), is_First_Image_received(false),
                                                                        detector_(hessian),sensor_id_("NO_ID"), frame_id_("empty"), matcher_(0.10),
                                                                        collisions_threshold_(0.10), mode_(0), weight_(1), is_disabled_(false){
  ROS_INFO("ROSFaultDetection Constructor");

  //Subscribers
  image_sub_ = nh.subscribe("camera", 3, &ROSFaultDetection::imageCb,this);
  ROS_INFO_STREAM("Camera topic " << image_sub_.getTopic());

  //Standalone
  image_pub_ = nh.advertise<sensor_msgs::Image>("Image", 1);
  cusum_pub_ = nh.advertise<std_msgs::Float64>("cusum_diff", 1);

  ros::NodeHandle nh2("~");
  //Sensor Fusion
  int number = 1;
  nh.getParam("sensor_number", number);
  ROS_INFO_STREAM("Publishing to topic " << "/collisions_" + std::to_string(number));
  output_msg_pub_ = nh2.advertise<fusion_msgs::sensorFusionMsg>("/collisions_" + std::to_string(number), 1);
  //dynamic_reconfigure
  dyn_server_cb = boost::bind(&ROSFaultDetection::dyn_reconfigureCB, this, _1, _2);
  dyn_server.setCallback(dyn_server_cb);
  nh.getParam("sensor_id", sensor_id_);
  ros::spin();
};


void ROSFaultDetection::reset(){

  last_cusum_ = 0.0;
  cusum_= 0,0;
  last_cusum_mean_ = 0.0;
  last_cusum_var_ = 1.0;
  current_.clearFrame();
  last_.clearFrame();
  is_First_Image_received = false;

}

void ROSFaultDetection::dyn_reconfigureCB(vision_utils_ros::dynamic_reconfigureConfig &config, uint32_t level){
  detector_.hessianThreshold = config.hessian_threshold;
  matcher_.setMatchPercentage(config.matching_threshold);
  collisions_threshold_ = config.collisions_threshold;
  weight_ = config.sensor_weight;
  mode_ = config.mode;
  is_disabled_ = config.disable;

  output_msg_pub_.shutdown();
  ros::NodeHandle nh2("~");
  output_msg_pub_ = nh2.advertise<fusion_msgs::sensorFusionMsg>("/collisions_" + std::to_string(config.detector_id), 1);

  if (config.reset){
    reset();
    config.reset = false;
  }
}

ROSFaultDetection::~ROSFaultDetection(){

};

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
    std_msgs::Header h = msg->header;
    frame_id_ = h.frame_id;
    ROS_INFO("First Frame Received");
    last_ = current_;

  }
  else{
    run();
    last_ = current_;
  }
};

void ROSFaultDetection::run(){

   if (mode_ == 0){
     matcher_.clearing();
     matcher_.matchD(current_,last_);
     matcher_.separateMatches(current_,last_);
     matcher_.getBestMatches(current_,last_);
     matcher_.separateBestMatches(current_,last_);
     matcher_.drawBestMatches(current_,last_);
     cusum_ = statics_tool->CUSUM(matcher_, last_cusum_mean_, last_cusum_var_, last_cusum_);
   }
   else{
     cusum_  = statics_tool->getBlur(current_.getFrame());
   }

   if (!is_disabled_){
     publishOutputs();
   }
};

void ROSFaultDetection::publishOutputs(){

 // Publishing image after matching
 Mat img = current_.getFrame();
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
 out_msg_2.data =  cusum_;
 cusum_pub_.publish(out_msg_2);

 //sensorFusionMsg
 output_msg_.header.stamp = ros::Time::now();
 output_msg_.header.frame_id = frame_id_;
 output_msg_.sensor_id.data = sensor_id_.c_str();
 output_msg_.data.clear();

 //double focusMeasure = detectBlur();
 output_msg_.data.push_back(cusum_-last_cusum_);
 //output_msg_.data.push_back(focusMeasure);
 output_msg_.window_size = 1;

 output_msg_.weight = weight_;

 if (fabs(cusum_- last_cusum_) >= collisions_threshold_){
   output_msg_.msg = fusion_msgs::sensorFusionMsg::ERROR;
 }
 else{
   output_msg_.msg = fusion_msgs::sensorFusionMsg::OK;
 }
 last_cusum_ = cusum_;
 output_msg_pub_.publish(output_msg_);
};

double ROSFaultDetection::detectBlur(){
 cv::Mat lap;
 cv::Laplacian(current_.getFrame(), lap, CV_64F);

 cv::Scalar mu, sigma;
 cv::meanStdDev(lap, mu, sigma);

 double focusMeasure = sigma.val[0]*sigma.val[0];
 return focusMeasure;
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

};
