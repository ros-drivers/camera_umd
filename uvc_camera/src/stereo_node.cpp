#include <ros/ros.h>
#include <nodelet/loader.h>

int main (int argc, char **argv) {
  ros::init(argc, argv, "uvc_camera_stereo");

  nodelet::Loader nodelet;
  nodelet::M_string remap(ros::names::getRemappings());
  nodelet::V_string nargv;

  nodelet.load("uvc_camera_stereo", "uvc_camera/StereoNodelet", remap, nargv);

  ros::spin();
  return 0;
}

