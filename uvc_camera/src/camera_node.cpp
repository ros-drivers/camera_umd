#include <ros/ros.h>
#include "uvc_camera/camera.h"

namespace uvc_camera {

class CameraNode {
  public:
    CameraNode() : camera(ros::NodeHandle(), ros::NodeHandle("~")) {
    }

  private:
    Camera camera;
};

};

int main (int argc, char **argv) {
  ros::init(argc, argv, "uvc_camera");
  ros::NodeHandle nh;

  uvc_camera::CameraNode cn;

  ros::spin();
}

