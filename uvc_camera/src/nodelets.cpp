#include <ros/ros.h>
#include <ros/time.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "uvc_camera/camera.h"
#include "uvc_camera/stereocamera.h"

namespace uvc_camera {

class CameraNodelet : public nodelet::Nodelet {
  public:
    CameraNodelet() {}

    void onInit() {
      ros::NodeHandle node = getNodeHandle();
      ros::NodeHandle pnode = getPrivateNodeHandle();

      camera = new Camera(node, pnode);
    }

    ~CameraNodelet() {
      if (camera) delete camera;
    }

  private:
    Camera *camera;
};

class StereoNodelet : public nodelet::Nodelet {
  public:
    StereoNodelet() {}

    void onInit() {
      ros::NodeHandle node = getNodeHandle();
      ros::NodeHandle pnode = getPrivateNodeHandle();

      stereo = new StereoCamera(node, pnode);
    }

    ~StereoNodelet() {
      if (stereo) delete stereo;
    }

  private:
    StereoCamera *stereo;
};

};

PLUGINLIB_EXPORT_CLASS(uvc_camera::CameraNodelet, nodelet::Nodelet)
PLUGINLIB_EXPORT_CLASS(uvc_camera::StereoNodelet, nodelet::Nodelet)

