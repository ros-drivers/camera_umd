/* JPEG CompressedImage -> HTTP streaming node for ROS
 * (c) 2010 Ken Tossell / ktossell@umd.edu
 */
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <image_transport/image_transport.h>
#include <boost/thread.hpp>
#include <boost/tuple/tuple.hpp>
#include <boost/lexical_cast.hpp>
#include "mongoose.h"

using namespace std;

class JPEGStreamer {
  public:
    JPEGStreamer();
    void add_connection(struct mg_connection *con, boost::condition_variable *cond, boost::mutex *single_mutex);

  private:
    ros::NodeHandle node;
    ros::Subscriber image_sub;
    mg_context *web_context;
    boost::mutex con_mutex;
    boost::mutex data_mutex;
    list<boost::tuple<struct mg_connection*, boost::condition_variable*, boost::mutex*> > connections;

    void image_callback(const sensor_msgs::CompressedImage::ConstPtr& msg);

    int skip, skipped;
};

static JPEGStreamer *g_status_video;

void *new_req_callback(enum mg_event event, struct mg_connection *con, const struct mg_request_info *req) {
  boost::condition_variable cond;
  boost::mutex single_mutex;
  boost::unique_lock<boost::mutex> lock(single_mutex);
  g_status_video->add_connection(con, &cond, &single_mutex);
  cond.wait(lock);
  return (void*) 1;
}

JPEGStreamer::JPEGStreamer() {
  string topic = node.resolveName("image");
  int port, start_threads, max_threads;
  ros::NodeHandle("~").param("port", port, 8080);
  ros::NodeHandle("~").param("skip", skip, 0);
  ros::NodeHandle("~").param("start_threads", start_threads, 1);
  ros::NodeHandle("~").param("max_threads", max_threads, 16);
  skipped = 0;

  std::stringstream port_ss, num_threads_ss, max_threads_ss;
  port_ss << port;
  num_threads_ss << start_threads;
  max_threads_ss << max_threads;

  const char *mg_options[] = {
    "listening_ports", strdup(port_ss.str().c_str()),
    "num_threads", strdup(num_threads_ss.str().c_str()),
    "max_threads", strdup(max_threads_ss.str().c_str()),
    "authentication_domain", ".",
    NULL
  };

  image_sub = node.subscribe<sensor_msgs::CompressedImage>(topic, 1,
    boost::bind(&JPEGStreamer::image_callback, this, _1)
  );

  g_status_video = this;

  web_context = mg_start(&new_req_callback, NULL, mg_options);
}

static char header_text[] = "HTTP/1.0 200 OK\r\nConnection: Close\r\n"
  "Server: ros/jpeg_streamer\r\n"
  "Content-Type: multipart/x-mixed-replace;boundary=--myboundary\r\n\r\n";

void JPEGStreamer::add_connection(struct mg_connection *con, boost::condition_variable *cond, boost::mutex *single_mutex) {
  mg_write(con, header_text, sizeof(header_text));

  {
    boost::mutex::scoped_lock lock(con_mutex);

    connections.push_back(boost::tuple<struct mg_connection*, boost::condition_variable*, boost::mutex*>(con, cond, single_mutex));
  }
}

void JPEGStreamer::image_callback(const sensor_msgs::CompressedImage::ConstPtr& msg) {
  if (skipped++ == skip)
    skipped = 0;
  else
    return;

  string data = "--myboundary\r\nContent-Type: image/jpeg\r\nContent-Length: "
    + boost::lexical_cast<string>(msg->data.size()) + "\r\n\r\n";

  data.append((const char*) &(msg->data[0]), msg->data.size());

  data += "\r\n";

  const char *buf = data.c_str();
  int buf_len = data.length();

  /* Send frame to our subscribers */
  {
    boost::mutex::scoped_lock con_lock(con_mutex);

    for (list<boost::tuple<struct mg_connection*, boost::condition_variable*, boost::mutex*> >::iterator it = connections.begin();
         it != connections.end(); it++) {
      struct mg_connection *con = (*it).get<0>();
      boost::condition_variable *cond = (*it).get<1>();
      // boost::mutex *single_mutex = (*it).get<2>();

      if (mg_write(con, buf, buf_len) < buf_len) {
        it = connections.erase(it);
        cond->notify_one();
      }
    }
  }
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "jpeg_streamer");

  JPEGStreamer streamer;

  ros::spin();

  return 0;
}
