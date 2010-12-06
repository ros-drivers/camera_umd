#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include <pluginlib/class_list_macros.h>
#include <nodelet/nodelet.h>

#include "uvc_cam/uvc_cam.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"
#include "image_transport/image_transport.h"

using namespace sensor_msgs;

namespace uvc_camera {

class Camera : public nodelet::Nodelet {
  public:
    Camera() {}

    void onInit() {
      /* make local copies of the node references */
      node = getNodeHandle();
      pnode = getPrivateNodeHandle();
      it = new image_transport::ImageTransport(node);

      /* default config values */
      width = 640;
      height = 480;
      fps = 10;
      skip_frames = 0;
      frames_to_skip = 0;
      device = "/dev/video0";
      frame = "camera";
      rotate = false;

      /* set up information manager */
      info_mgr = new CameraInfoManager(node, "camera");

      std::string url;

      pnode.getParam("camera_info_url", url);

      info_mgr->loadCameraInfo(url);

      /* pull other configuration */
      pnode.getParam("device", device);

      pnode.getParam("fps", fps);
      pnode.getParam("skip_frames", skip_frames);

      pnode.getParam("width", width);
      pnode.getParam("height", height);

      pnode.getParam("frame_id", frame);

      /* advertise image streams and info streams */
      pub = it->advertise("image_raw", 1);

      info_pub = node.advertise<CameraInfo>("camera_info", 1);

      /* initialize the cameras */
      cam = new uvc_cam::Cam(device.c_str(), uvc_cam::Cam::MODE_RGB, width, height, fps);
      cam->set_motion_thresholds(100, -1);

      /* and turn on the streamer */
      ok = true;
      image_thread = boost::thread(boost::bind(&Camera::feedImages, this));
    }

    void sendInfo(ros::Time time) {
      CameraInfoPtr info(new CameraInfo(info_mgr->getCameraInfo()));

      info->header.stamp = time;
      info->header.frame_id = frame;

      info_pub.publish(info);
    }

    void feedImages() {
      unsigned int pair_id = 0;
      while (ok) {
        unsigned char *img_frame = NULL;
        uint32_t bytes_used;

        ros::Time capture_time = ros::Time::now();

        int idx = cam->grab(&img_frame, bytes_used);

        /* Read in every frame the camera generates, but only send each
         * (skip_frames + 1)th frame. It's set up this way just because
         * this is based on Stereo...
         */
        if (skip_frames == 0 || frames_to_skip == 0) {
          if (img_frame) {
             ImagePtr image(new Image);

             image->height = height;
             image->width = width;
             image->step = 3 * width;
             image->encoding = image_encodings::RGB8;

             image->header.stamp = capture_time;
             image->header.seq = pair_id;

             image->header.frame_id = frame;

             image->data.resize(image->step * image->height);

             memcpy(&image->data[0], img_frame, width*height * 3);

             pub.publish(image);

             sendInfo(capture_time);

             ++pair_id;
          }

          frames_to_skip = skip_frames;
        } else {
          frames_to_skip--;
        }

        if (img_frame) cam->release(idx);
      }
    }

    ~Camera() {
      ok = false;
      image_thread.join();
      if (it) delete it;
      if (cam) delete cam;
    }

  private:
    ros::NodeHandle node, pnode;
    image_transport::ImageTransport *it;
    bool ok;

    uvc_cam::Cam *cam;
    int width, height, fps, skip_frames, frames_to_skip;
    std::string device, frame;
    bool rotate;

    CameraInfoManager *info_mgr;

    image_transport::Publisher pub;
    ros::Publisher info_pub;

    boost::thread image_thread;
};

PLUGINLIB_DECLARE_CLASS(uvc_camera, Camera, uvc_camera::Camera, nodelet::Nodelet);

};

/*
int main (int argc, char **argv) {
  ros::init(argc, argv, "uvc_stereo");

  Stereo uvc_node(argc, argv);

  boost::thread image_thread(boost::bind(&Stereo::feedImages, &uvc_node));

  ros::spin();
}
*/
