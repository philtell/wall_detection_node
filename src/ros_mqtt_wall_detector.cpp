#include <ros/ros.h>
#include "cloud_process.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "wall_detector_node");
  ros::NodeHandle nh;
  
  GroundSegmentation processor(nh); // 正确初始化
  
  ros::AsyncSpinner spinner(2); // 使用多线程spinner
  spinner.start();
  ros::waitForShutdown();
  
  return 0;
}