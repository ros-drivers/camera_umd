#include <ros/ros.h>
#include "uvc_camera/stereocamera.h"


namespace uvc_camera {

class StereoNode {
  public:
    StereoNode() : stereo(ros::NodeHandle(), ros::NodeHandle("~")) {
    }

  private:
    StereoCamera stereo;
};

};

int main (int argc, char **argv) {
  ros::init(argc, argv, "uvc_camera_stereo");
  ros::NodeHandle nh;

  uvc_camera::StereoNode sn;

  ros::spin();
}

