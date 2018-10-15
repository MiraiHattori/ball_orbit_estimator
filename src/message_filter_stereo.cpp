#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

#include <boost/thread.hpp>
#include <limits>
#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>

#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

#include <std_msgs/Time.h>
#include <geometry_msgs/PointStamped.h>
#include <opencv_apps/MomentArrayStamped.h>
#include <opencv_apps/Point2DStamped.h>
#include <opencv_apps/Point2DArrayStamped.h>

namespace orbit_estimation
{
class SyncStereoImagesNodelet : public nodelet::Nodelet
{
public:
  // Constructor
  SyncStereoImagesNodelet() = default;

  ~SyncStereoImagesNodelet()
  {
    boost::mutex::scoped_lock scoped_lock(connect_mutex_);

    if (pub_thread_)
    {
      pub_thread_->interrupt();
      pub_thread_->join();
    }
  }

private:
  void connectCb()
  {
    boost::mutex::scoped_lock scoped_lock(connect_mutex_);
    if (pub_point_.getNumSubscribers() == 0 and pub_pixel_.getNumSubscribers() == 0 and
        rpub_pixel_.getNumSubscribers() == 0 and pub_pixels_.getNumSubscribers() == 0)
    {
      if (pub_thread_)
      {
        pub_thread_->interrupt();
        scoped_lock.unlock();
        pub_thread_->join();
        scoped_lock.lock();
        pub_thread_.reset();
      }
    }
    else if (not pub_thread_)
    {
      pub_thread_.reset(new boost::thread(boost::bind(&SyncStereoImagesNodelet::loop, this)));
    }
  }

  virtual void onInit()
  {
    ros::NodeHandle& pnh = getMTPrivateNodeHandle();
    pnh.param("approximate_sync", approximate_sync_, true);
    pnh.param("queue_size", queue_size_, 100);

    boost::mutex::scoped_lock scoped_lock(connect_mutex_);
    ros::SubscriberStatusCallback cb = boost::bind(&SyncStereoImagesNodelet::connectCb, this);
    pub_point_ = getMTNodeHandle().advertise<geometry_msgs::PointStamped>("/pointgrey/ball_point", 1, cb, cb);
    pub_pixels_ = getMTNodeHandle().advertise<opencv_apps::Point2DArrayStamped>("/pointgrey/ball_pixels", 1, cb, cb);
    pub_pixel_ = getMTNodeHandle().advertise<opencv_apps::Point2DStamped>("/pointgrey/left/ball_pixel", 1, cb, cb);
    rpub_pixel_ = getMTNodeHandle().advertise<opencv_apps::Point2DStamped>("/pointgrey/right/ball_pixel", 1, cb, cb);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr&)
  {
    // this is dummy; we need this to acquire camera_info
  }

  void infoCallback(const sensor_msgs::CameraInfoConstPtr& msg, const bool& is_left)
  {
    sensor_msgs::CameraInfo ci = *msg;
    if (is_left and not ci_)
    {
      ci_.reset(new sensor_msgs::CameraInfo(ci));
    }
    else if (not is_left and not rci_)
    {
      rci_.reset(new sensor_msgs::CameraInfo(ci));
    }
  }

  void momentsCallback(const boost::shared_ptr<opencv_apps::MomentArrayStamped const>& moments,
                       const boost::shared_ptr<opencv_apps::MomentArrayStamped const>& rmoments)
  {
    boost::this_thread::disable_interruption no_interruption{};
    boost::mutex::scoped_lock scoped_lock(connect_mutex_);

    if (not moments or not rmoments)
    {
      std::cerr << "moments or rmoments is nullptr " << std::endl;
      return;
    }
    if (moments->moments.empty() or rmoments->moments.empty())
    {
      // std::cerr << "moments are empty" << std::endl;
      return;
    }
    // getting each ball position in rectified camera pixel
    auto center_it = std::max_element(
        moments->moments.begin(), moments->moments.end(),
        [](const opencv_apps::Moment& m1, const opencv_apps::Moment& m2) { return m1.area < m2.area; });
    double center_x = center_it->center.x;
    double center_y = center_it->center.y;
    std_msgs::Time stamp;
    stamp.data = moments->header.stamp;
    auto rcenter_it = std::max_element(
        rmoments->moments.begin(), rmoments->moments.end(),
        [](const opencv_apps::Moment& m1, const opencv_apps::Moment& m2) { return m1.area < m2.area; });
    double rcenter_x = rcenter_it->center.x;
    double rcenter_y = rcenter_it->center.y;
    std_msgs::Time rstamp;
    rstamp.data = rmoments->header.stamp;
    // std::cerr << "(" << center_x << ", " << center_y << ") "
    //           << "(" << rcenter_x << ", " << rcenter_y << ") " << std::endl;
    float pd[12] = { static_cast<float>(ci_->P[0]), static_cast<float>(ci_->P[1]),  static_cast<float>(ci_->P[2]),
                     static_cast<float>(ci_->P[3]), static_cast<float>(ci_->P[4]),  static_cast<float>(ci_->P[5]),
                     static_cast<float>(ci_->P[6]), static_cast<float>(ci_->P[7]),  static_cast<float>(ci_->P[8]),
                     static_cast<float>(ci_->P[9]), static_cast<float>(ci_->P[10]), static_cast<float>(ci_->P[11]) };

    cv::Mat p(cv::Size(4, 3), CV_32F, pd);
    std::vector<cv::Point2f> xy;
    // xy.push_back(cv::Point2f(2, 3));
    xy.push_back(cv::Point2f(center_x, center_y));

    // float rpd[12] = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 };
    // clang-format off
    float rpd[12] = { static_cast<float>(rci_->P[0]), static_cast<float>(rci_->P[1]), static_cast<float>(rci_->P[2]),
                      // static_cast<float>(rci_->P[3]) * 1000.0f / static_cast<float>(rci_->P[0]),
                      static_cast<float>(rci_->P[3]), static_cast<float>(rci_->P[4]), static_cast<float>(rci_->P[5]),
                      static_cast<float>(rci_->P[6]), static_cast<float>(rci_->P[7]), static_cast<float>(rci_->P[8]),
                      static_cast<float>(rci_->P[9]), static_cast<float>(rci_->P[10]), static_cast<float>(rci_->P[11]) };
    // clang-format on
    cv::Mat rp(cv::Size(4, 3), CV_32F, rpd);
    std::vector<cv::Point2f> rxy;
    // rxy.push_back(cv::Point2f(1, 2));
    rxy.push_back(cv::Point2f(rcenter_x, rcenter_y));
    float resultd[4] = { 0.0, 0.0, 0.0, 0.0 };
    cv::Mat result(cv::Size(1, 4), CV_32F, resultd);
    cv::triangulatePoints(p, rp, xy, rxy, result);
    std_msgs::Header header;
    header.seq = seq_;
    header.stamp = stamp.data;
    header.frame_id = "/pointgrey_tf_opt";
    seq_++;
    p_msg_.header = header;
    geometry_msgs::Point point;
    /*
    point.x = result.at<float>(0, 0) / result.at<float>(0, 3);
    point.y = result.at<float>(1, 0) / result.at<float>(0, 3);
    point.z = result.at<float>(2, 0) / result.at<float>(0, 3);
    */
    point.x = result.at<float>(0, 0) / result.at<float>(3, 0);
    point.y = result.at<float>(1, 0) / result.at<float>(3, 0);
    point.z = result.at<float>(2, 0) / result.at<float>(3, 0);
    p_msg_.point = point;
    pub_point_.publish(p_msg_);
    opencv_apps::Point2DStamped pixel;
    opencv_apps::Point2D pixel_left;
    pixel_left.x = center_x;
    pixel_left.y = center_y;
    pixel.header = header;
    pixel.point = pixel_left;
    opencv_apps::Point2DStamped rpixel;
    opencv_apps::Point2D pixel_right;
    pixel_right.x = rcenter_x;
    pixel_right.y = rcenter_y;
    rpixel.header = header;
    rpixel.point = pixel_right;
    pub_pixel_.publish(pixel);
    rpub_pixel_.publish(rpixel);
    std::vector<opencv_apps::Point2D> pixels({ pixel_left, pixel_right });
    opencv_apps::Point2DArrayStamped pixels_msg;
    pixels_msg.header = header;
    pixels_msg.points = pixels;
    pub_pixels_.publish(pixels_msg);
  }

  void loop()
  {
    // Image subscriber is dummy
    ros::Subscriber sub_cam_img = getMTNodeHandle().subscribe<sensor_msgs::Image>(
        "/pointgrey/left/image_raw", 10, boost::bind(&SyncStereoImagesNodelet::imageCallback, this, _1));
    ros::Subscriber rsub_cam_img = getMTNodeHandle().subscribe<sensor_msgs::Image>(
        "/pointgrey/right/image_raw", 10, boost::bind(&SyncStereoImagesNodelet::imageCallback, this, _1));
    ros::Subscriber sub_cam_info = getMTNodeHandle().subscribe<sensor_msgs::CameraInfo>(
        "/pointgrey/left/camera_info", 10, boost::bind(&SyncStereoImagesNodelet::infoCallback, this, _1, true));
    ros::Subscriber rsub_cam_info = getMTNodeHandle().subscribe<sensor_msgs::CameraInfo>(
        "/pointgrey/right/camera_info", 10, boost::bind(&SyncStereoImagesNodelet::infoCallback, this, _1, false));

    // getting camera info
    ros::Rate loop(10);
    while (ros::ok())
    {
      if (ci_ and rci_)
      {
        sub_cam_info.shutdown();
        rsub_cam_info.shutdown();
        break;
      }
      loop.sleep();
    }

    std::unique_ptr<message_filters::Subscriber<opencv_apps::MomentArrayStamped>> sub;
    sub.reset(new message_filters::Subscriber<opencv_apps::MomentArrayStamped>(getMTNodeHandle(),
                                                                               "/pointgrey/left/centroid/moments", 1));
    std::unique_ptr<message_filters::Subscriber<opencv_apps::MomentArrayStamped>> rsub;
    rsub.reset(new message_filters::Subscriber<opencv_apps::MomentArrayStamped>(
        getMTNodeHandle(), "/pointgrey/right/centroid/moments", 1));
    using MyApproxSyncPolicy = message_filters::sync_policies::ApproximateTime<opencv_apps::MomentArrayStamped,
                                                                               opencv_apps::MomentArrayStamped>;
    std::unique_ptr<message_filters::Synchronizer<MyApproxSyncPolicy>> approximate_synchronizer;
    using MyExactSyncPolicy =
        message_filters::sync_policies::ExactTime<opencv_apps::MomentArrayStamped, opencv_apps::MomentArrayStamped>;
    std::unique_ptr<message_filters::Synchronizer<MyExactSyncPolicy>> exact_synchronizer;
    if (approximate_sync_)
    {
      std::cerr << "Approximate Synchronizer policy is selected" << std::endl;
      approximate_synchronizer.reset(
          new message_filters::Synchronizer<MyApproxSyncPolicy>(MyApproxSyncPolicy(queue_size_), *sub, *rsub));
      approximate_synchronizer->registerCallback(boost::bind(&SyncStereoImagesNodelet::momentsCallback, this, _1, _2));
    }
    else
    {
      std::cerr << "Exact Synchronizer policy is selected" << std::endl;
      exact_synchronizer.reset(
          new message_filters::Synchronizer<MyExactSyncPolicy>(MyExactSyncPolicy(queue_size_), *sub, *rsub));
      exact_synchronizer->registerCallback(boost::bind(&SyncStereoImagesNodelet::momentsCallback, this, _1, _2));
    }
    pub_thread_->yield();
    boost::this_thread::disable_interruption no_interruption{};
    ros::Rate loop2(1);
    while (not boost::this_thread::interruption_requested())
    {
      if (pub_point_.getNumSubscribers() == 0 and pub_pixel_.getNumSubscribers() == 0 and
          rpub_pixel_.getNumSubscribers() == 0 and pub_pixels_.getNumSubscribers() == 0)
      {
        break;
      }
      loop2.sleep();
    }
  }

  int seq_ = 0;
  geometry_msgs::PointStamped p_msg_;
  ros::Publisher pub_point_;
  ros::Publisher pub_pixels_;
  ros::Publisher pub_pixel_;
  ros::Publisher rpub_pixel_;
  std::unique_ptr<sensor_msgs::CameraInfo> ci_ = nullptr;
  std::unique_ptr<sensor_msgs::CameraInfo> rci_ = nullptr;

  boost::shared_ptr<boost::thread> pub_thread_;
  boost::mutex connect_mutex_;

  bool approximate_sync_ = true;
  int queue_size_ = 10;

  ros::NodeHandle pnh_;
};

PLUGINLIB_DECLARE_CLASS(orbit_estimation, SyncStereoImagesNodelet, orbit_estimation::SyncStereoImagesNodelet,
                        nodelet::Nodelet);
}
