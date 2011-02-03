#include <ros/ros.h>
#include <nodelet/loader.h>

int main (int argc, char **argv) {
  ros::init(argc, argv, "uvc_camera");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;

  nodelet.load("uvc_camera", "uvc_camera/CameraNodelet", remap, nargv);

  ros::spin();
  return 0;
}

