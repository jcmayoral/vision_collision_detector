#include <vision_collision_detector/ros_fault_detection.h>

int main(int argc, char *argv[]){
    ros::init(argc, argv, "vision_fault_detection");
    ros::NodeHandle nh("~");
    ROSFaultDetection fd_(nh);
};
