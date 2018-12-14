#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

#include <boost/array.hpp>
#include <boost/thread.hpp>
#include <memory>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <ball_state_msgs/PosAndVelWithCovarianceStamped.h>
#include <opencv_apps/Point2DArrayStamped.h>

#include <opencv2/calib3d/calib3d.hpp>
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Core>
#include <Eigen/LU>
#include <Eigen/Geometry>

#include "ball_orbit_estimator/ekf.h"

namespace ball_orbit_estimator
{
/*
 * @brief groundのtfから見たボールの軌道をpublishするnodelet
 * 左右眼画像2枚を受け取り，ボールの予測軌道をEKFで返す
 */
class OrbitEstimationNodelet : public nodelet::Nodelet
{
public:
  // Constructor
  OrbitEstimationNodelet() = default;

  ~OrbitEstimationNodelet()
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
    if (pub_ball_state_.getNumSubscribers() == 0)
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
      pub_thread_.reset(new boost::thread(boost::bind(&OrbitEstimationNodelet::loop, this)));
    }
  }

  virtual void onInit()
  {
    boost::mutex::scoped_lock scoped_lock(connect_mutex_);
    ros::SubscriberStatusCallback cb = boost::bind(&OrbitEstimationNodelet::connectCb, this);
    pub_ball_state_ = getMTNodeHandle().advertise<ball_state_msgs::PosAndVelWithCovarianceStamped>("/pointgrey/estimated_ball_state", 1, cb, cb);

    // EKFの初期化はコールバック内で行う
  }

  void callback(const boost::shared_ptr<opencv_apps::Point2DArrayStamped const>& pixels)
  {
    if (pixels->points.size() != 2)
    {
      std::cerr << "[ball_orbit_estimator] invalid points size in ekf_ball_orbit_estimator.cpp" << std::endl;
    }

    // {{{ getting camera tf
    tf::StampedTransform camera_tf;
    try
    {
#warning using ground tf as origin ロボットが回転したらどうなるのか検討する
      // カメラ姿勢を取得してtransform
      camera_tf_listener_.lookupTransform("/pointgrey_tf", "ground", ros::Time(0), camera_tf);
    }
    catch (tf::TransformException& ex)
    {
      ROS_ERROR("%s", ex.what());
      return;
    }
    tf::Quaternion tf_q_camera = camera_tf.getRotation();
    tf::Vector3 tf_pos_camera = camera_tf.getOrigin();
    Eigen::Vector3d pos_camera;
    tf::vectorTFToEigen(tf_pos_camera, pos_camera);
    Eigen::Vector3d pos_camera_inv = -pos_camera;
    Eigen::Quaterniond q_camera;
    tf::quaternionTFToEigen(tf_q_camera, q_camera);
    Eigen::Quaterniond q_camera_inv = q_camera.inverse();
    // }}}

    // {{{ getting raw point

    Eigen::VectorXd pixel_l(2);
    Eigen::VectorXd pixel_r(2);
    pixel_l[0] = pixels->points.at(0).x;
    pixel_l[1] = pixels->points.at(0).y;
    pixel_r[0] = pixels->points.at(1).x;
    pixel_r[1] = pixels->points.at(1).y;

    Eigen::MatrixXd PL(3, 4);
    Eigen::MatrixXd PR(3, 4);
    // clang-format off
    PL <<  ci_->P[0], ci_->P[1], ci_->P[2], ci_->P[3],
           ci_->P[4], ci_->P[5], ci_->P[6], ci_->P[7],
           ci_->P[8], ci_->P[9], ci_->P[10], ci_->P[11];
    PR <<  rci_->P[0], rci_->P[1], rci_->P[2], rci_->P[3],
           rci_->P[4], rci_->P[5], rci_->P[6], rci_->P[7],
           rci_->P[8], rci_->P[9], rci_->P[10], rci_->P[11];
    // clang-format on
    float pd[12] = {
      static_cast<float>(PL(0, 0)), static_cast<float>(PL(0, 1)), static_cast<float>(PL(0, 2)),
      static_cast<float>(PL(0, 3)), static_cast<float>(PL(1, 0)), static_cast<float>(PL(1, 1)),
      static_cast<float>(PL(1, 2)), static_cast<float>(PL(1, 3)), static_cast<float>(PL(2, 0)),
      static_cast<float>(PL(2, 1)), static_cast<float>(PL(2, 2)), static_cast<float>(PL(2, 3)),
    };
    cv::Mat p(cv::Size(4, 3), CV_32F, pd);
    std::vector<cv::Point2f> xy;
    xy.push_back(cv::Point2f(static_cast<float>(pixel_l[0]), static_cast<float>(pixel_l[1])));
    float rpd[12] = {
      static_cast<float>(PR(0, 0)), static_cast<float>(PR(0, 1)), static_cast<float>(PR(0, 2)),
      static_cast<float>(PR(0, 3)), static_cast<float>(PR(1, 0)), static_cast<float>(PR(1, 1)),
      static_cast<float>(PR(1, 2)), static_cast<float>(PR(1, 3)), static_cast<float>(PR(2, 0)),
      static_cast<float>(PR(2, 1)), static_cast<float>(PR(2, 2)), static_cast<float>(PR(2, 3)),
    };
    cv::Mat rp(cv::Size(4, 3), CV_32F, rpd);
    std::vector<cv::Point2f> rxy;
    rxy.push_back(cv::Point2f(static_cast<float>(pixel_r[0]), static_cast<float>(pixel_r[1])));
    float resultd[4] = { 0.0, 0.0, 0.0, 0.0 };
    cv::Mat result(cv::Size(1, 4), CV_32F, resultd);
    cv::triangulatePoints(p, rp, xy, rxy, result);

    // 光学座標系でのボール位置
    Eigen::VectorXd point_opt(3);
    // リンク座標系でのボール位置
    Eigen::VectorXd point(3);
    Eigen::VectorXd point_rot(3);
    point_opt << result.at<float>(0, 0) / result.at<float>(3, 0), result.at<float>(1, 0) / result.at<float>(3, 0),
        result.at<float>(2, 0) / result.at<float>(3, 0);
    point << point_opt[2], -point_opt[0], -point_opt[1];
    point_rot = q_camera * point + pos_camera;
    // リンク座標を地面の姿勢に変えた系でのボール位置
    std::cerr << "measured: " << point_rot[0] << " " << point_rot[1] << " " << point_rot[2] << std::endl;

    std_msgs::Header header = pixels->header;
    header.frame_id = "ground";
    if (not is_time_initialized_)
    {
      t_ = header.stamp;
      is_time_initialized_ = true;
    }
    ros::Time t = t_;
    t_ = header.stamp;
    ros::Duration diff = t_ - t;
    double delta_t = diff.toSec();
    if (delta_t < 0.0)
    {
      std::cerr << "ball_orbit_estimator: orbit estimation time stamp is inverted" << std::endl;
    }
    // }}}

    // {{{ EKF
    /*
     * 状態: ボールの(x, y, z, vx, vy, vz)
     * 撮影時刻の前回からのdiff(delta_t)
     * カメラ位置(pos_camera)
     * カメラ姿勢(q_camera)と画面上でのボール中心(lx, ly, rx, ry)のあるピクセルEKF
     */
    const Eigen::VectorXd GRAVITY((Eigen::VectorXd(3) << 0, 0, -9.80665).finished());
    // 状態xはground座標系でのボールの位置と速度を6次元並べたもの
    Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
    F.block(0, 3, 3, 3) = delta_t * Eigen::MatrixXd::Identity(3, 3);

    std::function<Eigen::VectorXd(Eigen::VectorXd)> f = [F](Eigen::VectorXd x) { return F * x; };
    Eigen::MatrixXd G = Eigen::MatrixXd::Identity(6, 6);
    // 誤差を入れた(入れないと正定値性失う可能性)
    Eigen::MatrixXd Q = 0.01 * Eigen::MatrixXd::Identity(6, 6);
    Eigen::VectorXd u(6);
    u.segment(0, 3) = GRAVITY * delta_t * delta_t / 2.0;
    u.segment(3, 3) = GRAVITY * delta_t;
    Eigen::VectorXd z(4);
    z << pixel_l[0], pixel_l[1], pixel_r[0], pixel_r[1];
    std::function<Eigen::VectorXd(Eigen::VectorXd)> h = [PL, PR, q_camera_inv, pos_camera_inv](Eigen::VectorXd x) {
      Eigen::VectorXd z_(4);
      // x.segment(0, 3) extracts ball pos
      Eigen::VectorXd x_rot = q_camera_inv * x.segment(0, 3) + pos_camera_inv;
      double X = x_rot[0];
      double Y = x_rot[1];
      double Z = x_rot[2];
      z_ << -PL(0, 0) * Y / X + PL(0, 2), -PL(1, 1) * Z / X + PL(1, 2), -PR(0, 0) * Y / X + PR(0, 2) + PR(0, 3) / X,
          -PR(1, 1) * Z / X + PR(1, 2);
      return z_;
    };
    std::function<Eigen::MatrixXd(Eigen::VectorXd)> dh = [PL, PR, q_camera_inv,
                                                          pos_camera_inv](Eigen::VectorXd x_filtered_pre) {
      // x_filtered_pre.segment(0, 3) extracts ball pos
      Eigen::VectorXd x_rot = q_camera_inv * x_filtered_pre.segment(0, 3) + pos_camera_inv;
      double X = x_rot[0];
      double Y = x_rot[1];
      double Z = x_rot[2];
      Eigen::MatrixXd H = Eigen::MatrixXd::Zero(4, 6);
      H(0, 0) = PL(0, 0) * Y / (X * X);
      H(0, 1) = -PL(0, 0) / X;
      H(1, 0) = PL(1, 1) * Z / (X * X);
      H(1, 2) = -PL(1, 1) / X;
      H(2, 0) = PR(0, 0) * Y / (X * X) - PR(0, 3) / (X * X);
      H(2, 1) = -PR(0, 0) / X;
      H(3, 0) = PR(1, 1) * Z / (X * X);
      H(3, 2) = -PR(1, 1) / X;
      return H;
    };
    // 画素のばらつき
    Eigen::MatrixXd R = 2.0 * Eigen::MatrixXd::Identity(4, 4);

    if (not is_ekf_initialized_)
    {
      Eigen::VectorXd x_init(6);
#warning 初期値をもっとしっかりフィルタをかける
      // 雑な値を入れておく(位置は最初に見つけたところ，速度は45度射出時に原点に向かうところ)
      // v0*t=distance, v0*t-g*t^2/2=-zより，v0=d*g^0.5/(2d-2z)^0.5 (g=-GRAVITY[2])
      double distance = std::hypot(point_rot[0], point_rot[1]);
      double v0 = distance * std::sqrt(-GRAVITY[2]) / std::sqrt(2 * (std::abs(distance + point_rot[2])));
      x_init << point_rot[0], point_rot[1], point_rot[2],                    // 位置
          -v0 * point_rot[0] / distance, -v0 * point_rot[1] / distance, v0;  // 速度
      // 雑な値を入れておいたので増やしておく
      Eigen::MatrixXd P_init(6, 6);
      // clang-format off
      P_init << 1.0, 0.0, 0.0, 100.0, 0.0, 0.0,
                0.0, 0.5, 0.0, 0.0, 100.0, 0.0,
                0.0, 0.0, 2.0, 0.0, 0.0, 100.0,
                100.0, 0.0, 0.0, 5.0, 0.0, 0.0,
                0.0, 100.0, 0.0, 0.0, 1.0, 0.0,
                0.0, 0.0, 100.0, 0.0, 0.0, 3.0;
      // clang-format on
      is_ekf_initialized_ = true;
      ekf.reset(new Filter::EKF(x_init, P_init));
    }

    if (is_ekf_initialized_)
    {
      std::pair<Eigen::VectorXd, Eigen::MatrixXd> value = ekf->update(f, F, G, Q, u, z, h, dh, R);

      ball_state_msgs::PosAndVelWithCovarianceStamped state;
      state.header = header;
      geometry_msgs::Point point_msg;
      point_msg.x = (value.first)[0];
      point_msg.y = (value.first)[1];
      point_msg.z = (value.first)[2];
      state.point = point_msg;
      geometry_msgs::Vector3 velocity_msg;
      velocity_msg.x = (value.first)[3];
      velocity_msg.y = (value.first)[4];
      velocity_msg.z = (value.first)[5];
      state.velocity = velocity_msg;
      boost::array<double, 36> cov_msg;  // 位置速度を並べた6次元ベクトル
      for (size_t i = 0; i < 6; i++)
      {
        for (size_t j = 0; j < 6; j++)
        {
          cov_msg.at(i * 6 + j) = (value.second)(i, j);
        }
      }
      state.pos_and_vel_covariance = cov_msg;
      pub_ball_state_.publish(state);

      // std::cerr << "estimated: " << (value.first)[0] << " " << (value.first)[1] << " " << (value.first)[2] << " "
      //           << (value.first)[3] << " " << (value.first)[4] << " " << (value.first)[5] << std::endl;
      // }}}
    }
  }

  // {{{ member functions to get camera info
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
  // }}}

  void loop()
  {
    // {{{ getting camera info
    // Image subscriber is dummy
    ros::Subscriber sub_cam_img = getMTNodeHandle().subscribe<sensor_msgs::Image>(
        "/pointgrey/left/image_raw", 10, boost::bind(&OrbitEstimationNodelet::imageCallback, this, _1));
    ros::Subscriber rsub_cam_img = getMTNodeHandle().subscribe<sensor_msgs::Image>(
        "/pointgrey/right/image_raw", 10, boost::bind(&OrbitEstimationNodelet::imageCallback, this, _1));
    ros::Subscriber sub_cam_info = getMTNodeHandle().subscribe<sensor_msgs::CameraInfo>(
        "/pointgrey/left/camera_info", 10, boost::bind(&OrbitEstimationNodelet::infoCallback, this, _1, true));
    ros::Subscriber rsub_cam_info = getMTNodeHandle().subscribe<sensor_msgs::CameraInfo>(
        "/pointgrey/right/camera_info", 10, boost::bind(&OrbitEstimationNodelet::infoCallback, this, _1, false));
    ros::Rate loop(10);
    while (ros::ok())
    {
      if (ci_ and rci_)
      {
        sub_cam_img.shutdown();
        rsub_cam_img.shutdown();
        sub_cam_info.shutdown();
        rsub_cam_info.shutdown();
        break;
      }
      loop.sleep();
    }
    // }}}

    sub_pixels_ = getMTNodeHandle().subscribe<opencv_apps::Point2DArrayStamped>(
        "/pointgrey/ball_pixels", 1, boost::bind(&OrbitEstimationNodelet::callback, this, _1));
    pub_thread_->yield();
    boost::this_thread::disable_interruption no_interruption{};
    ros::Rate loop2(1);
    while (not boost::this_thread::interruption_requested())
    {
      if (pub_ball_state_.getNumSubscribers() == 0)
      {
        break;
      }
      loop2.sleep();
    }
  }

  boost::shared_ptr<boost::thread> pub_thread_;
  boost::mutex connect_mutex_;

  tf::TransformListener camera_tf_listener_;
  ros::Subscriber sub_pixels_;
  ros::Publisher pub_ball_state_;
  std::unique_ptr<sensor_msgs::CameraInfo> ci_ = nullptr;
  std::unique_ptr<sensor_msgs::CameraInfo> rci_ = nullptr;
  bool is_ekf_initialized_ = false;
  std::unique_ptr<Filter::EKF> ekf = nullptr;

  bool is_time_initialized_ = false;
  ros::Time t_;
};

PLUGINLIB_DECLARE_CLASS(ball_orbit_estimator, OrbitEstimationNodelet, ball_orbit_estimator::OrbitEstimationNodelet,
                        nodelet::Nodelet);
}
