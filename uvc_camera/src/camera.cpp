#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include "uvc_cam/uvc_cam.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"
#include "image_transport/image_transport.h"

#include "uvc_camera/camera.h"

using namespace sensor_msgs;

namespace uvc_camera {

Camera::Camera(ros::NodeHandle _comm_nh, ros::NodeHandle _param_nh) :
      node(_comm_nh), pnode(_param_nh), it(_comm_nh),
      info_mgr(_comm_nh, "camera"), cam(0) {

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
      std::string url;

      pnode.getParam("camera_info_url", url);

      info_mgr.loadCameraInfo(url);

      /* pull other configuration */
      pnode.getParam("device", device);

      pnode.getParam("fps", fps);
      pnode.getParam("skip_frames", skip_frames);

      pnode.getParam("width", width);
      pnode.getParam("height", height);

      pnode.getParam("frame_id", frame);

      /* advertise image streams and info streams */
      pub = it.advertise("image_raw", 1);

      info_pub = node.advertise<CameraInfo>("camera_info", 1);

      /* initialize the cameras */
      cam = new uvc_cam::Cam(device.c_str(), uvc_cam::Cam::MODE_RGB, width, height, fps);
      cam->set_motion_thresholds(100, -1);

      /* and turn on the streamer */
      ok = true;
      image_thread = boost::thread(boost::bind(&Camera::feedImages, this));
    }

    void Camera::sendInfo(ImagePtr &image, ros::Time time) {
      CameraInfoPtr info(new CameraInfo(info_mgr.getCameraInfo()));

      /* Throw out any CamInfo that's not calibrated to this camera mode */
      if (info->K[0] != 0.0 &&
           (image->width != info->width
              || image->height != info->height)) {
        info.reset(new CameraInfo());
      }

      /* If we don't have a calibration, set the image dimensions */
      if (info->K[0] == 0.0) {
        info->width = image->width;
        info->height = image->height;
      }

      info->header.stamp = time;
      info->header.frame_id = frame;

      info_pub.publish(info);
    }

    void Camera::feedImages() {
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

             sendInfo(image, capture_time);

             ++pair_id;
          }

          frames_to_skip = skip_frames;
        } else {
          frames_to_skip--;
        }

        if (img_frame) cam->release(idx);
      }
    }

    Camera::~Camera() {
      ok = false;
      image_thread.join();
      if (cam) delete cam;
    }


};

