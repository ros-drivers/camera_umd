#include <boost/thread.hpp>

#include <ros/ros.h>
#include <ros/time.h>

#include "uvc_cam/uvc_cam.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/image_encodings.h"
#include "sensor_msgs/CameraInfo.h"
#include "camera_info_manager/camera_info_manager.h"
#include "image_transport/image_transport.h"

using namespace sensor_msgs;

/* Rotate an 8-bit, 3-channel image by 180 degrees. */
static inline void rotate(unsigned char *dst_chr, unsigned char *src_chr, int pixels) {
  struct pixel_t { unsigned char r, g, b; };

  struct pixel_t *src = (pixel_t*) src_chr;
  struct pixel_t *dst = &(((pixel_t*) dst_chr)[pixels - 1]);

  for (int i = pixels; i != 0; --i) {
    *dst = *src;
    src++;
    dst--;
  }
}

class UVCStereo {
  public:
    UVCStereo(int argc, char **argv) : pnode("~"), it(node),
      width(640), height(480),
      fps(10), skip_frames(0), frames_to_skip(0),
      left_device("/dev/video0"), right_device("/dev/video1"),
      frame("camera"),
      rotate_left(false), rotate_right(false),
      left_info_mgr(ros::NodeHandle(node, "left"), "left_camera"),
      right_info_mgr(ros::NodeHandle(node, "right"), "right_camera") {

      std::string left_url, right_url;

      pnode.getParam("left/camera_info_url", left_url);
      pnode.getParam("right/camera_info_url", right_url);

      left_info_mgr.loadCameraInfo(left_url);
      right_info_mgr.loadCameraInfo(right_url);

      pnode.getParam("left/device", left_device);
      pnode.getParam("right/device", right_device);

      pnode.getParam("fps", fps);
      pnode.getParam("skip_frames", skip_frames);

      pnode.getParam("left/rotate", rotate_left);
      pnode.getParam("right/rotate", rotate_right);

      pnode.getParam("width", width);
      pnode.getParam("height", height);

      pnode.getParam("frame_id", frame);

      left_pub = it.advertise("left/image_raw", 1);
      right_pub = it.advertise("right/image_raw", 1);

      left_info_pub = node.advertise<CameraInfo>("left/camera_info", 1);
      right_info_pub = node.advertise<CameraInfo>("right/camera_info", 1);

      cam_left = new uvc_cam::Cam(left_device.c_str(), uvc_cam::Cam::MODE_RGB, width, height, fps);
      cam_left->set_motion_thresholds(100, -1);
      cam_right = new uvc_cam::Cam(right_device.c_str(), uvc_cam::Cam::MODE_RGB, width, height, fps);
      cam_right->set_motion_thresholds(100, -1);
    }

    void sendInfo(ros::Time time) {
      CameraInfo info_left = left_info_mgr.getCameraInfo();
      CameraInfo info_right = right_info_mgr.getCameraInfo();

      info_left.header.stamp = info_right.header.stamp = time;
      info_left.header.frame_id = frame;
      info_right.header.frame_id = frame;

      left_info_pub.publish(info_left);
      right_info_pub.publish(info_right);
    }

    void feedImages() {
      unsigned int pair_id = 0;
      while (node.ok()) {
        unsigned char *frame_left = NULL, *frame_right = NULL;
        uint32_t bytes_used_left, bytes_used_right;

        ros::Time capture_time = ros::Time::now();

        int left_idx = cam_left->grab(&frame_left, bytes_used_left);
        int right_idx = cam_right->grab(&frame_right, bytes_used_right);

        /* Read in every frame the camera generates, but only send each
         * (skip_frames + 1)th frame. This reduces effective frame
         * rate, processing time and network usage while keeping the
         * images synchronized within the true framerate.
         */
        if (skip_frames == 0 || frames_to_skip == 0) {
          if (frame_left && frame_right) {
             Image image_left, image_right;

             image_left.height = height;
             image_left.width = width;
             image_left.step = 3 * width;
             image_left.encoding = image_encodings::RGB8;

             image_right = image_left;

             image_left.header.stamp = capture_time;
             image_left.header.seq = pair_id;
             image_right.header.stamp = capture_time;
             image_right.header.seq = pair_id;

             image_left.header.frame_id = frame;
             image_right.header.frame_id = frame;

             image_left.data.resize(image_left.step * image_left.height);
             image_right.data.resize(image_right.step * image_right.height);

             if (rotate_left)
               rotate(&image_left.data[0], frame_left, width*height);
             else
               memcpy(&image_left.data[0], frame_left, width*height * 3);

             if (rotate_right)
               rotate(&image_right.data[0], frame_right, width*height);
             else
               memcpy(&image_right.data[0], frame_right, width*height * 3);

             left_pub.publish(image_left);
             right_pub.publish(image_right);

             sendInfo(capture_time);

             ++pair_id;
          }

          frames_to_skip = skip_frames;
        } else {
          frames_to_skip--;
        }

        if (frame_left) cam_left->release(left_idx);
        if (frame_right) cam_right->release(right_idx);
      }
    }

    ~UVCStereo() {
      delete cam_left;
      delete cam_right;
    }

  private:
    ros::NodeHandle node, pnode;
    image_transport::ImageTransport it;

    uvc_cam::Cam *cam_left, *cam_right;
    int width, height, fps, skip_frames, frames_to_skip;
    std::string left_device, right_device, frame;
    bool rotate_left, rotate_right;

    camera_info_manager::CameraInfoManager left_info_mgr, right_info_mgr;

    image_transport::Publisher left_pub, right_pub;
    ros::Publisher left_info_pub, right_info_pub;

};

int main (int argc, char **argv) {
  ros::init(argc, argv, "uvc_stereo");

  UVCStereo uvc_node(argc, argv);

  boost::thread image_thread(boost::bind(&UVCStereo::feedImages, &uvc_node));

  ROS_WARN("This package has been replaced. Please use the "
      "uvc_camera package's stereo_node or StereoNodelet instead.");

  ros::spin();
}
