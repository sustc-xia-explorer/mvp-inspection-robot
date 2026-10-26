/*
 * neupan_cpp_ros: ROS1 Noetic wrapper for libneupan, the C++ port of NeuPAN.
 *
 * Ported from neupan_ros (https://github.com/hanruihua/neupan_ros),
 * Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. See <https://www.gnu.org/licenses/>.
 */

#include <ros/ros.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/exceptions.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include "neupan/neupan_planner.hpp"
#include "neupan/command_limiter.hpp"

namespace {

double quatToYaw(const geometry_msgs::Quaternion& q) {
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                    1.0 - 2.0 * (q.z * q.z + q.y * q.y));
}

geometry_msgs::Quaternion yawToQuat(double yaw) {
  geometry_msgs::Quaternion q;
  q.z = std::sin(yaw / 2.0);
  q.w = std::cos(yaw / 2.0);
  return q;
}

}  // namespace

class NeuPANNode {
 public:
  NeuPANNode() : nh_(), pnh_("~"), tf_listener_(tf_buffer_) {
    std::string config_file, dune_checkpoint;
    pnh_.param<std::string>("config_file", config_file, "");
    pnh_.param<std::string>("dune_checkpoint", dune_checkpoint, "");
    pnh_.param<std::string>("map_frame", map_frame_, "map");
    pnh_.param<std::string>("base_frame", base_frame_, "base_link");
    pnh_.param<std::string>("lidar_frame", lidar_frame_, "laser_link");
    pnh_.param("marker_size", marker_size_, 0.05);
    pnh_.param("marker_z", marker_z_, 0.3);
    pnh_.param("scan_angle_min", scan_angle_min_, -3.141593);
    pnh_.param("scan_angle_max", scan_angle_max_, 3.141593);
    pnh_.param("scan_range_min", scan_range_min_, 0.1);
    pnh_.param("scan_range_max", scan_range_max_, 5.0);
    pnh_.param("scan_downsample", scan_downsample_, 1);
    pnh_.param("flip_angle", flip_angle_, false);
    pnh_.param("include_initial_path_direction", include_initial_path_direction_, false);
    pnh_.param("scan_timeout", scan_timeout_, 0.5);
    pnh_.param("pose_timeout", pose_timeout_, 0.5);
    pnh_.param("solver_fail_grace", solver_fail_grace_, 0);
    pnh_.param("stall_speed", stall_speed_, 0.02);
    pnh_.param("stall_timeout", stall_timeout_, 3.0);
    double rate;
    pnh_.param("control_rate", rate, 10.0);
    if (!std::isfinite(rate) || rate <= 0.0 ||
        !std::isfinite(scan_timeout_) || scan_timeout_ <= 0.0 ||
        !std::isfinite(pose_timeout_) || pose_timeout_ <= 0.0 ||
        !std::isfinite(scan_range_min_) || !std::isfinite(scan_range_max_) ||
        scan_range_min_ < 0.0 || scan_range_max_ <= scan_range_min_ ||
        solver_fail_grace_ < 0)
      throw std::runtime_error("invalid rate, timeout, scan range or solver grace");
    last_progress_time_ = ros::Time::now();

    if (scan_downsample_ < 1)
      throw std::runtime_error("scan_downsample must be >= 1");

    if (config_file.empty())
      throw std::runtime_error("parameter 'config_file' is required");
    if (dune_checkpoint.empty())
      throw std::runtime_error(
          "parameter 'dune_checkpoint' (.bin model) is required");

    planner_ = std::make_unique<neupan::NeuPANPlanner>(
        neupan::NeuPANPlanner::fromYaml(config_file, dune_checkpoint));

    vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/neupan_cmd_vel", 1);
    plan_pub_ = nh_.advertise<nav_msgs::Path>("/neupan_plan", 1);
    ref_state_pub_ = nh_.advertise<nav_msgs::Path>("/neupan_ref_state", 1);
    ref_path_pub_ = nh_.advertise<nav_msgs::Path>("/neupan_initial_path", 1);
    dune_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/dune_point_markers", 1);
    nrmp_markers_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/nrmp_point_markers", 1);
    robot_marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/robot_marker", 1);
    arrive_pub_ = nh_.advertise<std_msgs::Bool>("/neupan_arrive", 1);
    diag_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("/neupan_diagnostics", 1);
    scan_sub_ = nh_.subscribe<sensor_msgs::LaserScan>(
        "/scan", 1, &NeuPANNode::scanCallbackRos, this,
        ros::TransportHints().tcpNoDelay());
    path_sub_ = nh_.subscribe("/initial_path", 1, &NeuPANNode::pathCallbackRos, this);
    waypoints_sub_ = nh_.subscribe("/neupan_waypoints", 1, &NeuPANNode::waypointsCallbackRos, this);
    goal_sub_ = nh_.subscribe("/neupan_goal", 1, &NeuPANNode::goalCallbackRos, this);
    emergency_pub_ = nh_.advertise<std_msgs::Bool>("/neupan_emergency_stop", 1, true);
    std::string applied_topic;
    pnh_.param<std::string>("applied_cmd_topic", applied_topic, "");
    if (!applied_topic.empty())
      applied_sub_ = nh_.subscribe<geometry_msgs::TwistStamped>(applied_topic, 1,
          &NeuPANNode::appliedCallback, this);
    last_output_time_ = ros::WallTime::now();
    timer_ = nh_.createTimer(ros::Duration(1.0 / rate), &NeuPANNode::timerCallback, this);
    ROS_INFO("neupan_cpp ROS1 ready (config: %s)", config_file.c_str());
  }

  ~NeuPANNode() {
    vel_pub_.publish(geometry_msgs::Twist());
  }

 private:
  void scanCallbackRos(const sensor_msgs::LaserScan::ConstPtr& msg) { scanCallback(*msg); }
  void pathCallbackRos(const nav_msgs::Path::ConstPtr& msg) { pathCallback(*msg); }
  void waypointsCallbackRos(const nav_msgs::Path::ConstPtr& msg) { waypointsCallback(*msg); }
  void goalCallbackRos(const geometry_msgs::PoseStamped::ConstPtr& msg) { goalCallback(*msg); }

  void appliedCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    if (msg->header.frame_id != base_frame_) return;
    const double age = (ros::Time::now()-msg->header.stamp).toSec();
    if (!std::isfinite(age) || age < -.1 || age > .3) return;
    const neupan::Vec2 cmd(msg->twist.linear.x, msg->twist.angular.z);
    try { planner_->setAppliedCommand(cmd); }
    catch (const std::exception& ex) { ROS_WARN_THROTTLE(1., "%s", ex.what()); }
  }

  void timerCallback(const ros::TimerEvent&) {
    try {
      run();
    } catch (const std::exception& ex) {
      halt(ex.what());
    }
  }

  std::optional<neupan::Vec3> lookupPose(const std::string& target,
                                        const std::string& source) {
    try {
      const auto tfs = tf_buffer_.lookupTransform(target, source, ros::Time(0),
                                                 ros::Duration(0.02));
      const double age = (ros::Time::now() - tfs.header.stamp).toSec();
      if (!tfs.header.stamp.isZero() && (age > pose_timeout_ || age < -0.1)) {
        ROS_WARN_THROTTLE(1.0, "robot TF is stale or from the future");
        return std::nullopt;
      }
      return neupan::Vec3(tfs.transform.translation.x,
                          tfs.transform.translation.y,
                          quatToYaw(tfs.transform.rotation));
    } catch (const tf2::TransformException& ex) {
      ROS_WARN_THROTTLE(1.0, "waiting for tf %s -> %s: %s",
                        target.c_str(), source.c_str(), ex.what());
      return std::nullopt;
    }
  }

  bool inMapFrame(const std::string& frame) const {
    if (frame == map_frame_) return true;
    ROS_WARN("path/goal frame '%s' must match map_frame '%s'",
             frame.c_str(), map_frame_.c_str());
    return false;
  }

  // Publishing nothing is not a stop: consumers hold the last command.
  void halt(const char* why) {
    vel_pub_.publish(geometry_msgs::Twist());
    last_output_.setZero();
    planner_->setAppliedCommand(last_output_);
    last_output_time_ = ros::WallTime::now();
    std_msgs::Bool emergency;
    emergency.data = true;
    emergency_pub_.publish(emergency);
    publishArrive(false);
    ROS_WARN_THROTTLE(1.0, "%s", why);
  }

  void run() {
    const auto state = lookupPose(map_frame_, base_frame_);
    if (!state) {
      halt("no robot pose (TF map->base), holding still");
      return;
    }
    robot_state_ = *state;
    have_state_ = true;

    if (scanIsStale()) {
      obstacle_points_.resize(2, 0);
      halt("scan is stale, obstacles unknown, holding still");
      return;
    }

    auto& ipath = planner_->ipath();
    if (ipath.hasConfiguredWaypoints() && !ipath.hasPath())
      ipath.setIpathWithState(robot_state_);

    if (!ipath.hasPath()) {
      halt("waiting for neupan initial path");
      return;
    }

    publishInitialPath();

    if (obstacle_points_.cols() == 0)
      ROS_WARN_THROTTLE(
          1.0,
          "no obstacle points, only path tracking will be performed");

    const auto tick = ros::WallTime::now();
    const double command_dt = std::max(1e-6, std::min(
        (tick-last_output_time_).toSec(), planner_->config().step_time));
    planner_->setCommandInterval(command_dt);
    neupan::NeuPANPlanner::Info info;
    const neupan::Vec2 action =
        planner_->forward(robot_state_, obstacle_points_, info);

    if (info.arrive)
      ROS_INFO_THROTTLE(1.0,
                           "arrive at the target");
    if (info.stop)
      ROS_WARN_THROTTLE(0.5,
                           "neupan stop, min distance %.3f below threshold %.3f",
                           info.min_distance, planner_->collisionThreshold());

    geometry_msgs::Twist twist;
    const bool arriving = info.arrive || (info.braking && info.opt_s.cols() == 0);
    if (arriving) consecutive_unsolved_ = 0;
    else solverGaveUp(info);  // core supplies a bounded braking command
    neupan::Vec2 output = action;
    if (!info.stop) output = neupan::limitCommand(action, last_output_,
        planner_->config().max_speed, planner_->config().max_acce, command_dt);
    twist.linear.x = output(0);
    twist.angular.z = output(1);
    if (!std::isfinite(twist.linear.x) || !std::isfinite(twist.angular.z)) {
      halt("non-finite planner action");
      return;
    }
    last_output_ = output;
    last_output_time_ = ros::WallTime::now();
    planner_->setAppliedCommand(output);
    std_msgs::Bool emergency;
    emergency.data = info.stop;
    emergency_pub_.publish(emergency);
    vel_pub_.publish(twist);
    publishArrive(info.arrive && output.cwiseAbs().maxCoeff() < 1e-10);
    updateStall(twist, info.arrive);
    publishDiagnostics(info, twist);

    if (info.opt_s.cols() > 0) plan_pub_.publish(matToPath(info.opt_s));
    if (info.ref_s.cols() > 0) ref_state_pub_.publish(matToPath(info.ref_s));
    publishPointMarkers(info.dune_points, dune_markers_pub_, 160, 32, 240);
    publishPointMarkers(info.nrmp_points, nrmp_markers_pub_, 255, 128, 0);
    publishRobotMarker();
  }

  // A run of unsolved cycles means driving blind on a nominal with no avoidance.
  bool solverGaveUp(const neupan::NeuPANPlanner::Info& info) {
    if (info.solved) {
      consecutive_unsolved_ = 0;
      return false;
    }
    ++consecutive_unsolved_;
    if (consecutive_unsolved_ <= solver_fail_grace_) return false;

    ROS_WARN_THROTTLE(0.5,
                         "%d consecutive unsolved cycles (osqp status %d), stopping",
                         consecutive_unsolved_, info.solver_status);
    return true;
  }

  bool scanIsStale() const {
    if (!scan_seen_) return true;  // Require a valid scan before any motion.
    return (ros::Time::now() - last_scan_time_).toSec() > scan_timeout_;
  }

  void publishArrive(bool arrived) {
    std_msgs::Bool msg;
    msg.data = arrived;
    arrive_pub_.publish(msg);
  }

  // Idling at the clearance in front of a blocked path sets neither stop nor arrive.
  void updateStall(const geometry_msgs::Twist& cmd, bool arrived) {
    const bool moving = std::abs(cmd.linear.x) > stall_speed_ ||
                        std::abs(cmd.angular.z) > stall_speed_;
    if (moving || arrived) {
      last_progress_time_ = ros::Time::now();
      stalled_ = false;
      return;
    }
    if ((ros::Time::now() - last_progress_time_).toSec() > stall_timeout_) {
      stalled_ = true;
      ROS_WARN_THROTTLE(2.0,
                           "stalled for %.1f s without arriving; the path may "
                           "be blocked and need a global replan",
                           stall_timeout_);
    }
  }

  void publishDiagnostics(const neupan::NeuPANPlanner::Info& info,
                          const geometry_msgs::Twist& cmd) {
    diagnostic_msgs::DiagnosticStatus st;
    st.name = "neupan";
    st.hardware_id = base_frame_;
    if (info.arrive || (info.braking && info.opt_s.cols() == 0)) {
      st.level = diagnostic_msgs::DiagnosticStatus::OK;
      st.message = info.arrive ? "arrived" : "braking at goal";
    } else if (stalled_ || consecutive_unsolved_ > solver_fail_grace_) {
      st.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      st.message = stalled_ ? "stalled" : "solver failing";
    } else if (info.stop || !info.solved) {
      st.level = diagnostic_msgs::DiagnosticStatus::WARN;
      st.message = info.stop ? "stopped at collision threshold" : "unsolved";
    } else {
      st.level = diagnostic_msgs::DiagnosticStatus::OK;
      st.message = info.arrive ? "arrived" : "tracking";
    }

    const auto kv = [&](const char* k, const std::string& v) {
      diagnostic_msgs::KeyValue e;
      e.key = k;
      e.value = v;
      st.values.push_back(e);
    };
    kv("solved", info.solved ? "true" : "false");
    kv("solver_status", std::to_string(info.solver_status));
    kv("consecutive_unsolved", std::to_string(consecutive_unsolved_));
    kv("min_distance", std::to_string(info.min_distance));
    kv("obstacle_points", std::to_string(obstacle_points_.cols()));
    kv("stalled", stalled_ ? "true" : "false");
    kv("braking", info.braking ? "true" : "false");
    kv("emergency_stop", info.stop ? "true" : "false");
    kv("raw_v", std::to_string(info.raw_action(0)));
    kv("raw_w", std::to_string(info.raw_action(1)));
    kv("cmd_v", std::to_string(cmd.linear.x));
    kv("cmd_w", std::to_string(cmd.angular.z));

    diagnostic_msgs::DiagnosticArray arr;
    arr.header.stamp = ros::Time::now();
    arr.status.push_back(st);
    diag_pub_.publish(arr);
  }

  void scanCallback(const sensor_msgs::LaserScan& msg) {
    if (!have_state_) return;

    const std::string frame = msg.header.frame_id.empty() ? lidar_frame_ : msg.header.frame_id;
    const auto now = ros::Time::now();
    const double age = (now - msg.header.stamp).toSec();
    if (msg.ranges.empty() || (!msg.header.stamp.isZero() &&
        (age > scan_timeout_ || age < -0.1)) ||
        !std::isfinite(msg.angle_min) || !std::isfinite(msg.angle_max) ||
        !std::isfinite(msg.angle_increment)) {
      scan_seen_ = false;
      return;
    }
    geometry_msgs::TransformStamped transform;
    try {
      transform = tf_buffer_.lookupTransform(map_frame_, frame, msg.header.stamp,
                                              ros::Duration(0.02));
    } catch (const tf2::TransformException& ex) {
      scan_seen_ = false;
      ROS_WARN_THROTTLE(1.0, "scan TF unavailable: %s", ex.what());
      return;
    }
    const auto& q = transform.transform.rotation;
    const auto& t = transform.transform.translation;
    const tf2::Transform scan_to_map(
        tf2::Quaternion(q.x, q.y, q.z, q.w), tf2::Vector3(t.x, t.y, t.z));
    last_scan_time_ = now;
    scan_seen_ = true;

    const int n = static_cast<int>(msg.ranges.size());
    std::vector<double> xs, ys;
    xs.reserve(n);
    ys.reserve(n);

    // Deriving the increment from the range divides by one beam too few.
    const double angle_inc =
        msg.angle_increment != 0.0f
            ? msg.angle_increment
            : (n > 1 ? (msg.angle_max - msg.angle_min) / (n - 1) : 0.0);
    for (int i = 0; i < n; ++i) {
      if (i % scan_downsample_ != 0) continue;
      const double r = msg.ranges[i];
      const double angle = flip_angle_ ? msg.angle_max - i * angle_inc
                                       : msg.angle_min + i * angle_inc;
      if (std::isfinite(r) && r >= std::max(scan_range_min_, static_cast<double>(msg.range_min)) &&
          r <= std::min(scan_range_max_, static_cast<double>(msg.range_max)) &&
          angle > scan_angle_min_ && angle < scan_angle_max_) {
        xs.push_back(r * std::cos(angle));
        ys.push_back(r * std::sin(angle));
      }
    }

    if (xs.empty()) {
      obstacle_points_.resize(2, 0);
      return;
    }

    neupan::Mat2X pts(2, xs.size());
    for (size_t i = 0; i < xs.size(); ++i) {
      const tf2::Vector3 p = scan_to_map * tf2::Vector3(xs[i], ys[i], 0.0);
      pts(0, i) = p.x();
      pts(1, i) = p.y();
    }
    obstacle_points_ = std::move(pts);
  }

  // Path poses -> [x, y, theta, gear]; theta from the points gradient unless
  // include_initial_path_direction is set (as upstream).
  std::vector<neupan::InitialPath::PathPoint> pathMsgToPoints(
      const nav_msgs::Path& msg) const {
    std::vector<neupan::InitialPath::PathPoint> out;
    const size_t n = msg.poses.size();
    out.reserve(n);
    for (size_t i = 0; i < n; ++i) {
      const auto& p = msg.poses[i].pose;
      double theta;
      if (include_initial_path_direction_) {
        theta = quatToYaw(p.orientation);
      } else if (i + 1 < n) {
        const auto& p2 = msg.poses[i + 1].pose;
        theta = std::atan2(p2.position.y - p.position.y,
                           p2.position.x - p.position.x);
      } else {
        theta = out.empty() ? quatToYaw(p.orientation) : out.back()(2);
      }
      out.emplace_back(p.position.x, p.position.y, theta, 1.0);
    }
    return out;
  }

  void pathCallback(const nav_msgs::Path& msg) {
    if (!inMapFrame(msg.header.frame_id)) return;
    if (msg.poses.size() < 2) {
      ROS_WARN( "ignoring initial path with < 2 poses");
      return;
    }
    ROS_INFO_THROTTLE(1.0,
                         "initial path update from given path (%zu poses)",
                         msg.poses.size());

    // Replacing rather than resetting: a resetting replan is a cold start.
    if (planner_->ipath().hasPath() && have_state_) {
      planner_->replaceInitialPath(pathMsgToPoints(msg), robot_state_);
    } else {
      planner_->setInitialPath(pathMsgToPoints(msg));
      planner_->reset(false);
    }
    publishArrive(false);
  }

  void waypointsCallback(const nav_msgs::Path& msg) {
    if (!inMapFrame(msg.header.frame_id)) return;
    if (!have_state_ || msg.poses.empty()) return;
    std::vector<neupan::Vec3> wps{robot_state_};
    for (const auto& pp : pathMsgToPoints(msg)) wps.emplace_back(pp.head<3>());

    ROS_INFO_THROTTLE(1.0,
                         "initial path update from waypoints");
    planner_->setWaypoints(wps);
    planner_->reset(false);
    publishArrive(false);
  }

  void goalCallback(const geometry_msgs::PoseStamped& msg) {
    if (!inMapFrame(msg.header.frame_id)) return;
    if (!have_state_) return;
    const neupan::Vec3 goal(msg.pose.position.x, msg.pose.position.y,
                            quatToYaw(msg.pose.orientation));
    ROS_INFO( "set neupan goal: [%.2f, %.2f, %.2f]", goal(0),
                goal(1), goal(2));
    planner_->updateInitialPathFromGoal(robot_state_, goal);
    planner_->reset(false);
  }

  nav_msgs::Path matToPath(const neupan::Mat3X& states) {
    nav_msgs::Path path;
    path.header.frame_id = map_frame_;
    path.header.stamp = ros::Time::now();
    for (Eigen::Index i = 0; i < states.cols(); ++i) {
      geometry_msgs::PoseStamped ps;
      ps.header.frame_id = map_frame_;
      ps.pose.position.x = states(0, i);
      ps.pose.position.y = states(1, i);
      ps.pose.orientation = yawToQuat(states(2, i));
      path.poses.push_back(ps);
    }
    return path;
  }

  void publishInitialPath() {
    const auto& path = planner_->ipath().initialPath();
    nav_msgs::Path msg;
    msg.header.frame_id = map_frame_;
    msg.header.stamp = ros::Time::now();
    for (const auto& p : path) {
      geometry_msgs::PoseStamped ps;
      ps.header.frame_id = map_frame_;
      ps.pose.position.x = p(0);
      ps.pose.position.y = p(1);
      ps.pose.orientation = yawToQuat(p(2));
      msg.poses.push_back(ps);
    }
    ref_path_pub_.publish(msg);
  }

  void publishPointMarkers(
      const neupan::Mat2X& points,
      const ros::Publisher& pub,
      int r, int g, int b) {
    if (points.cols() == 0) return;
    visualization_msgs::MarkerArray arr;
    for (Eigen::Index i = 0; i < points.cols(); ++i) {
      visualization_msgs::Marker m;
      m.header.frame_id = map_frame_;
      m.header.stamp = ros::Time::now();
      m.id = static_cast<int>(i);
      m.type = visualization_msgs::Marker::CUBE;
      m.scale.x = m.scale.y = m.scale.z = marker_size_;
      m.color.a = 1.0;
      m.color.r = r / 255.0f;
      m.color.g = g / 255.0f;
      m.color.b = b / 255.0f;
      m.pose.position.x = points(0, i);
      m.pose.position.y = points(1, i);
      m.pose.position.z = 0.3;
      m.pose.orientation.w = 1.0;
      arr.markers.push_back(m);
    }
    pub.publish(arr);
  }

  void publishRobotMarker() {
    const auto& cfg = planner_->config();
    visualization_msgs::Marker m;
    m.header.frame_id = map_frame_;
    m.header.stamp = ros::Time::now();
    m.id = 0;
    m.type = visualization_msgs::Marker::CUBE;
    m.color.a = 1.0;
    m.color.g = 1.0;
    m.scale.x = cfg.length;
    m.scale.y = cfg.width;
    m.scale.z = marker_z_;
    m.pose.position.x = robot_state_(0);
    m.pose.position.y = robot_state_(1);
    m.pose.orientation = yawToQuat(robot_state_(2));
    robot_marker_pub_.publish(m);
  }

  neupan::Vec2 last_output_ = neupan::Vec2::Zero();
  ros::WallTime last_output_time_;
  ros::Publisher emergency_pub_;
  ros::Subscriber applied_sub_;
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;

  std::unique_ptr<neupan::NeuPANPlanner> planner_;

  std::string map_frame_, base_frame_, lidar_frame_;
  double marker_size_, marker_z_;
  double scan_angle_min_, scan_angle_max_, scan_range_min_, scan_range_max_;
  int scan_downsample_;
  bool flip_angle_, include_initial_path_direction_;
  double scan_timeout_, pose_timeout_, stall_speed_, stall_timeout_;
  int solver_fail_grace_ = 5;

  int consecutive_unsolved_ = 0;
  bool scan_seen_ = false, stalled_ = false;
  ros::Time last_scan_time_;
  ros::Time last_progress_time_;

  neupan::Vec3 robot_state_ = neupan::Vec3::Zero();
  bool have_state_ = false;
  neupan::Mat2X obstacle_points_ = neupan::Mat2X(2, 0);

  ros::Publisher vel_pub_, plan_pub_, ref_state_pub_, ref_path_pub_;
  ros::Publisher dune_markers_pub_, nrmp_markers_pub_, robot_marker_pub_;
  ros::Publisher arrive_pub_, diag_pub_;
  ros::Subscriber scan_sub_, path_sub_, waypoints_sub_, goal_sub_;
  ros::Timer timer_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "neupan_node");
  try {
    NeuPANNode node;
    ros::spin();
  } catch (const std::exception& ex) {
    ROS_FATAL("NeuPAN startup failed: %s", ex.what());
    return 1;
  }
  return 0;
}
