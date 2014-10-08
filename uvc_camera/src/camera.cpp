#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include "uvc_cam/uvc_cam.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
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
      format = "rgb";

      /* set up information manager */
      std::string url, camera_name;
      pnode.getParam("camera_info_url", url);
      pnode.param<std::string>("camera_name", camera_name, "camera");

      info_mgr.setCameraName(camera_name);
      info_mgr.loadCameraInfo(url);

      /* pull other configuration */
      pnode.getParam("device", device);

      pnode.getParam("fps", fps);
      pnode.getParam("skip_frames", skip_frames);

      pnode.getParam("width", width);
      pnode.getParam("height", height);

      pnode.getParam("frame_id", frame);

      pnode.getParam("format", format);

      /* advertise image streams and info streams */
      if (format != "jpeg")
        pub = it.advertise("image_raw", 1);
      else
        pubjpeg = node.advertise<CompressedImage>("image_raw/compressed", 1);

      info_pub = node.advertise<CameraInfo>("camera_info", 1);

      /* initialize the cameras */
      uvc_cam::Cam::mode_t mode = uvc_cam::Cam::MODE_RGB;
      if (format == "jpeg") 
        mode = uvc_cam::Cam::MODE_MJPG;      
      cam = new uvc_cam::Cam(device.c_str(), mode, width, height, fps);
      cam->set_motion_thresholds(100, -1);

      bool auto_focus;
      if (pnode.getParam("auto_focus", auto_focus)) {
        cam->set_v4l2_control(V4L2_CID_FOCUS_AUTO, auto_focus, "auto_focus");
      }

      int focus_absolute;
      if (pnode.getParam("focus_absolute", focus_absolute)) {
        cam->set_v4l2_control(V4L2_CID_FOCUS_ABSOLUTE, focus_absolute, "focus_absolute");
      }

      bool auto_exposure;
      if (pnode.getParam("auto_exposure", auto_exposure)) {
        int val;
        if (auto_exposure) {
          val = V4L2_EXPOSURE_AUTO;
        } else {
          val = V4L2_EXPOSURE_MANUAL;
        }
        cam->set_v4l2_control(V4L2_CID_EXPOSURE_AUTO, val, "auto_exposure");
      }

      int exposure_auto_priority;
      if (pnode.getParam("exposure_auto_priority", exposure_auto_priority)) {
        cam->set_v4l2_control(V4L2_CID_EXPOSURE_AUTO_PRIORITY, exposure_auto_priority, "exposure_auto_priority");
      } 

      int exposure_absolute;
      if (pnode.getParam("exposure_absolute", exposure_absolute)) {
        cam->set_v4l2_control(V4L2_CID_EXPOSURE_ABSOLUTE, exposure_absolute, "exposure_absolute");
      }

      int exposure;
      if (pnode.getParam("exposure", exposure)) {
        cam->set_v4l2_control(V4L2_CID_EXPOSURE, exposure, "exposure");
      }
      
      int brightness;
      if (pnode.getParam("brightness", brightness)) {
        cam->set_v4l2_control(V4L2_CID_BRIGHTNESS, brightness, "brightness");
      }

      int power_line_frequency;
      if (pnode.getParam("power_line_frequency", power_line_frequency)) {
        int val;
        if (power_line_frequency == 0) {
          val = V4L2_CID_POWER_LINE_FREQUENCY_DISABLED;
        } else if (power_line_frequency == 50) {
          val = V4L2_CID_POWER_LINE_FREQUENCY_50HZ;
        } else if (power_line_frequency == 60) {
          val = V4L2_CID_POWER_LINE_FREQUENCY_60HZ;
        } else {
          printf("power_line_frequency=%d not supported. Using auto.\n", power_line_frequency);
          val = V4L2_CID_POWER_LINE_FREQUENCY_AUTO;
        }
        cam->set_v4l2_control(V4L2_CID_POWER_LINE_FREQUENCY, val, "power_line_frequency");
      }

      int contrast;
      if (pnode.getParam("contrast", contrast)) {
        cam->set_v4l2_control(V4L2_CID_CONTRAST, contrast, "contrast");  
      }

      int saturation;
      if (pnode.getParam("saturation", saturation)) {
        cam->set_v4l2_control(V4L2_CID_SATURATION, saturation, "saturation"); 
      }

      int hue;
      if (pnode.getParam("hue", hue)) {
        cam->set_v4l2_control(V4L2_CID_HUE, hue, "hue"); 
      }

      bool auto_white_balance;
      if (pnode.getParam("auto_white_balance", auto_white_balance)) {
        cam->set_v4l2_control(V4L2_CID_AUTO_WHITE_BALANCE, auto_white_balance, "auto_white_balance");
      }

      int white_balance_tmp;
      if (pnode.getParam("white_balance_temperature", white_balance_tmp)) {   
        cam->set_v4l2_control(V4L2_CID_WHITE_BALANCE_TEMPERATURE, white_balance_tmp, "white_balance_temperature"); 
      }
      
      int gamma;
      if (pnode.getParam("gamma", gamma)) {
        cam->set_v4l2_control(V4L2_CID_GAMMA, gamma, "gamma");
      }

      int sharpness;
      if (pnode.getParam("sharpness", sharpness)) {
        cam->set_v4l2_control(V4L2_CID_SHARPNESS, sharpness, "sharpness");
      }

      int backlight_comp;
      if (pnode.getParam("backlight_compensation", backlight_comp)) {
        cam->set_v4l2_control(V4L2_CID_BACKLIGHT_COMPENSATION, backlight_comp, "backlight_compensation");
      }

      bool auto_gain;
      if (pnode.getParam("auto_gain", auto_gain)) {
        cam->set_v4l2_control(V4L2_CID_AUTOGAIN, auto_gain, "auto_gain");
      }

      int gain;
      if (pnode.getParam("gain", gain)) {
        cam->set_v4l2_control(V4L2_CID_GAIN, gain, "gain");
      }

      bool h_flip;
      if (pnode.getParam("horizontal_flip", h_flip)) {
        cam->set_v4l2_control(V4L2_CID_HFLIP, h_flip, "horizontal_flip");
      }

      bool v_flip;
      if (pnode.getParam("vertical_flip", v_flip)) {
        cam->set_v4l2_control(V4L2_CID_VFLIP, v_flip, "vertical_flip");
      }

      // TODO: 
      // - zoom absolute, zoom relative and zoom continuous controls
      // - add generic parameter list:
      //   [(id0, val0, name0), (id1, val1, name1), ...

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

    void Camera::sendInfoJpeg(ros::Time time) {
      CameraInfoPtr info(new CameraInfo(info_mgr.getCameraInfo()));
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
          if (img_frame && format != "jpeg") {
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
          } else if (img_frame && format == "jpeg") {
             CompressedImagePtr image(new CompressedImage);

             image->header.stamp = capture_time;
             image->header.seq = pair_id;

             image->header.frame_id = frame;

             image->data.resize(bytes_used);

             memcpy(&image->data[0], img_frame, bytes_used);

             pubjpeg.publish(image);

             sendInfoJpeg(capture_time);

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

