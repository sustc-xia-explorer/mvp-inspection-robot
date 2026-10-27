/*
 * neupan_cpp: C++ port of the NeuPAN planner.
 *
 * Ported from NeuPAN (https://github.com/hanruihua/NeuPAN),
 * Copyright (c) 2025 Ruihua Han <hanrh@connect.hku.hk>.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version. See <https://www.gnu.org/licenses/>.
 */

#include "neupan/initial_path.hpp"

#include <cmath>
#include <limits>
#include <stdexcept>

namespace neupan {

double wrapToPi(double rad) {
  while (rad > M_PI) rad -= 2.0 * M_PI;
  while (rad < -M_PI) rad += 2.0 * M_PI;
  return rad;
}

InitialPath::InitialPath(const Robot& robot, double ref_speed,
                         const Options& opts)
    : robot_(robot),
      T_(robot.T),
      dt_(robot.dt),
      ref_speed_(ref_speed),
      opts_(opts) {
  interval_ = opts.interval > 0.0 ? opts.interval : dt_ * ref_speed_;
  if (!opts_.waypoints.empty() && opts_.loop)
    throw std::runtime_error("initial_path: loop mode is not ported yet");
}

std::vector<InitialPath::PathPoint> InitialPath::generateLineCurve(
    const std::vector<Vec3>& waypoints) const {
  // Port of gctl curve_generator.generate_line + curve_from_waypoints
  // followed by initial_path._ensure_consistent_angles.
  std::vector<PathPoint> curve;
  curve.emplace_back(waypoints[0].x(), waypoints[0].y(), waypoints[0].z(),
                     1.0);

  for (size_t i = 0; i + 1 < waypoints.size(); ++i) {
    const Vec2 sp = waypoints[i].head<2>();
    const Vec2 ep = waypoints[i + 1].head<2>();
    const Vec2 diff = ep - sp;
    const double length = diff.norm();
    if (length < 1e-9) continue;  // skip duplicate/degenerate waypoints
    const Vec2 dir = diff / length;
    const double theta = std::atan2(diff.y(), diff.x());

    double cur_len = 0.0;
    while (cur_len + interval_ < length) {
      cur_len += interval_;
      const Vec2 p = sp + cur_len * dir;
      curve.emplace_back(p.x(), p.y(), theta, 1.0);
    }
    curve.emplace_back(ep.x(), ep.y(), waypoints[i + 1].z(), 1.0);
  }

  // _ensure_consistent_angles: theta = direction to the next point,
  // last point copies the second to last.
  for (size_t i = 0; i + 1 < curve.size(); ++i) {
    const double dx = curve[i + 1].x() - curve[i].x();
    const double dy = curve[i + 1].y() - curve[i].y();
    curve[i](2) = std::atan2(dy, dx);
  }
  if (curve.size() >= 2) curve[curve.size() - 1](2) = curve[curve.size() - 2](2);

  return curve;
}

void InitialPath::setPath(std::vector<PathPoint> path) {
  initial_path_ = std::move(path);
  splitPathWithGear();
  curve_index_ = 0;
  point_index_ = 0;
}

void InitialPath::splitPathWithGear() {
  curve_list_.clear();
  std::vector<PathPoint> current;
  double current_gear = initial_path_.front()(3);
  for (const auto& p : initial_path_) {
    if (p(3) != current_gear) {
      curve_list_.push_back(std::move(current));
      current.clear();
      current_gear = p(3);
    }
    current.push_back(p);
  }
  if (!current.empty()) curve_list_.push_back(std::move(current));
}

void InitialPath::setIpathWithState(const Vec3& state) {
  if (opts_.waypoints.empty())
    throw std::runtime_error("initial_path: waypoints are not set");
  std::vector<Vec3> wps;
  wps.push_back(state);
  wps.insert(wps.end(), opts_.waypoints.begin(), opts_.waypoints.end());
  setPath(generateLineCurve(wps));
}

void InitialPath::setInitialPath(std::vector<PathPoint> path) {
  if (path.size() < 2)
    throw std::invalid_argument(
        "initial_path: an external path needs at least 2 points");
  // Densify the external path to interval_, exactly like generateLineCurve.
  std::vector<Vec3> waypoints;
  waypoints.reserve(path.size());
  for (const auto& p : path) waypoints.emplace_back(p.head<3>());
  setPath(generateLineCurve(waypoints));
}

void InitialPath::replaceInitialPath(std::vector<PathPoint> path,
                                     const Vec3& state) {
  setInitialPath(std::move(path));

  double best = std::numeric_limits<double>::infinity();
  for (size_t c = 0; c < curve_list_.size(); ++c) {
    const auto& curve = curve_list_[c];
    for (size_t i = 0; i < curve.size(); ++i) {
      const double dis = (state.head<2>() - curve[i].head<2>()).norm();
      if (dis < best) {
        best = dis;
        curve_index_ = static_cast<int>(c);
        point_index_ = static_cast<int>(i);
      }
    }
  }
  arrive_flag_ = false;
}

void InitialPath::setIpathWithWaypoints(const std::vector<Vec3>& waypoints) {
  setPath(generateLineCurve(waypoints));
}

void InitialPath::updateInitialPathFromGoal(const Vec3& start,
                                            const Vec3& goal) {
  setPath(generateLineCurve({start, goal}));
}

void InitialPath::reset() {
  curve_index_ = 0;
  point_index_ = 0;
  arrive_flag_ = false;
}

double InitialPath::closestPoint(const Vec3& state, double threshold,
                                 int ind_range) {
  double min_dis = std::numeric_limits<double>::infinity();
  const auto& curve = curCurve();
  const int start = std::max(point_index_, 0);
  const int end =
      std::min<int>(point_index_ + ind_range, static_cast<int>(curve.size()));

  for (int i = start; i < end; ++i) {
    const double dis = (state.head<2>() - curve[i].head<2>()).norm();
    if (dis < min_dis) {
      min_dis = dis;
      point_index_ = i;
      if (dis < threshold) break;
    }
  }
  return min_dis;
}

bool InitialPath::checkCurveArrive(const Vec3& state) const {
  const auto& curve = curCurve();
  const double arrive_distance =
      (state.head<2>() - curve.back().head<2>()).norm();
  return arrive_distance < opts_.arrive_threshold &&
         point_index_ >= static_cast<int>(curve.size()) -
                             opts_.arrive_index_threshold - 2;
}

bool InitialPath::checkArrive(const Vec3& state) {
  if (!hasPath()) setIpathWithState(state);
  // Latch: recomputing every cycle flickers off when the robot settles past the end.
  if (arrive_flag_) return true;
  closestPoint(state, opts_.close_threshold, opts_.ind_range);

  if (checkCurveArrive(state)) {
    if (curve_index_ + 1 >= static_cast<int>(curve_list_.size())) {
      if (opts_.loop) {
        curve_index_ = 0;
        point_index_ = 0;
        return false;
      }
      arrive_flag_ = true;
      return true;
    }
    ++curve_index_;
    point_index_ = 0;
  }
  return false;
}

Vec3 InitialPath::findInteractionPoint(const Vec3& ref_state, int& ref_index,
                                       double length) const {
  const auto& curve = curCurve();
  const Vec2 circle = ref_state.head<2>();

  while (true) {
    if (ref_index > static_cast<int>(curve.size()) - 2) {
      Vec3 end = curve.back().head<3>();
      end(2) = wrapToPi(end(2));
      return end;
    }

    const PathPoint& cur = curve[ref_index];
    const PathPoint& next = curve[ref_index + 1];

    // range_cir_seg: circle-segment intersection, larger root only.
    const Vec2 sp = cur.head<2>(), ep = next.head<2>();
    const Vec2 d = ep - sp;
    bool found = false;
    Vec2 int_point;
    if (d.norm() != 0.0) {
      const Vec2 f = sp - circle;
      const double a = d.dot(d);
      const double b = 2.0 * f.dot(d);
      const double c = f.dot(f) - length * length;
      const double disc = b * b - 4.0 * a * c;
      if (disc >= 0.0) {
        const double t2 = (-b + std::sqrt(disc)) / (2.0 * a);
        if (t2 >= 0.0 && t2 <= 1.0) {
          int_point = sp + t2 * d;
          found = true;
        }
      }
    }

    if (found) {
      const double diff = wrapToPi(next(2) - cur(2));
      const double theta = wrapToPi(cur(2) + diff / 2.0);
      return Vec3(int_point.x(), int_point.y(), theta);
    }
    ++ref_index;
  }
}

Vec3 InitialPath::motionPredict(const Vec3& state, const Vec2& vel) const {
  return robot_.predict(state, vel);
}

InitialPath::NomRef InitialPath::generateNomRefState(
    const Vec3& state, const Mat2X& cur_vel_array, double ref_speed) const {
  const auto& curve = curCurve();
  const int n = static_cast<int>(curve.size());

  Vec3 ref_state = curve[point_index_].head<3>();
  int ref_index = point_index_;
  Vec3 pre_state = state;

  NomRef out;
  out.nom_s.resize(3, T_ + 1);
  out.ref_s.resize(3, T_ + 1);
  out.nom_u = cur_vel_array;
  out.ref_us.resize(T_);

  out.nom_s.col(0) = pre_state;
  out.ref_s.col(0) = ref_state;

  if (opts_.arc_length_sampling) {
    std::vector<double> arc(n, 0.0);
    for (int i = 1; i < n; ++i)
      arc[i] = arc[i-1] + (curve[i]-curve[i-1]).head<2>().norm();
    std::vector<double> speeds(n, ref_speed);
    if (opts_.turn_rate > 0.0) {
      std::vector<double> turn(n,0.0);
      for (int i=1; i<n; ++i)
        turn[i] = turn[i-1] + std::abs(wrapToPi(curve[i](2)-curve[i-1](2)));
      int left=0,right=0;
      for (int i=0; i<n; ++i) {
        while (left+1<i && arc[left+1]<arc[i]-opts_.turn_window/2.) ++left;
        right=std::max(right,i);
        while (right+1<n && arc[right]<arc[i]+opts_.turn_window/2.) ++right;
        const double curvature=(turn[right]-turn[left])/std::max(1e-9,arc[right]-arc[left]);
        speeds[i]=std::min(ref_speed,std::max(opts_.minimum_turn_speed,
                           opts_.turn_rate/std::max(1e-9,curvature)));
      }
      speeds.back()=0.0;
      // Braking before a corner, acceleration after it. The reference must be
      // dynamically attainable; clamping only the output reacts too late.
      const double accel=std::min(opts_.reference_acceleration,robot_.acce_bound(0)/dt_);
      for (int i=n-2; i>=0; --i)
        speeds[i]=std::min(speeds[i],std::sqrt(speeds[i+1]*speeds[i+1]+2.*accel*(arc[i+1]-arc[i])));
      for (int i=1; i<n; ++i)
        speeds[i]=std::min(speeds[i],std::sqrt(speeds[i-1]*speeds[i-1]+2.*accel*(arc[i]-arc[i-1])));
    }
    double start_arc = arc[point_index_];
    double best = std::numeric_limits<double>::infinity();
    // Continuous projection prevents reference jumps at the nearest point.
    for (int i = std::max(0, point_index_-1); i < std::min(n-1, point_index_+2); ++i) {
      const Vec2 a = curve[i].head<2>(), d = curve[i+1].head<2>()-a;
      const double fraction = d.squaredNorm() > 1e-16 ?
          std::max(0.0, std::min(1.0, (state.head<2>()-a).dot(d)/d.squaredNorm())) : 0.0;
      const double distance = (state.head<2>()-a-fraction*d).squaredNorm();
      if (distance < best) {
        best = distance;
        start_arc = arc[i] + fraction*(arc[i+1]-arc[i]);
      }
    }
    int index = std::max(0, point_index_-1);
    const auto sample = [&](double distance, int& j) -> Vec3 {
      while (j+1 < n-1 && arc[j+1] < distance) ++j;
      const double span = arc[j+1]-arc[j];
      const double fraction = span > 1e-12 ?
          std::max(0.0, std::min(1.0, (distance-arc[j])/span)) : 0.0;
      Vec3 value = curve[j].head<3>() + fraction*(curve[j+1]-curve[j]).head<3>();
      value(2) = curve[j](2) + fraction*wrapToPi(curve[j+1](2)-curve[j](2));
      return value;
    };
    out.ref_s.col(0) = sample(start_arc, index);
    double target_arc=start_arc;
    const auto speedAt = [&](double distance, int j) {
      while (j+1<n-1 && arc[j+1]<distance) ++j;
      const double fraction=std::max(0.,std::min(1.,(distance-arc[j])/std::max(1e-12,arc[j+1]-arc[j])));
      return speeds[j]+fraction*(speeds[j+1]-speeds[j]);
    };
    for (int t = 0; t < T_; ++t) {
      pre_state = motionPredict(pre_state, cur_vel_array.col(t));
      out.nom_s.col(t+1) = pre_state;
      const double speed=opts_.turn_rate>0.0 ? speedAt(target_arc,index) : ref_speed;
      target_arc = std::min(arc.back(),target_arc+speed*dt_);
      Vec3 reference = sample(target_arc, index);
      reference(2) = pre_state(2) + wrapToPi(reference(2)-pre_state(2));
      out.ref_s.col(t+1) = reference;
      out.ref_us(t) = target_arc < arc.back()-1e-9 ? speed*curve[index](3) : 0.0;
      if (robot_.kinematics == Kinematics::Omni && out.ref_us(t) != 0.0) {
        // Signed-speed and travel-angle pairs describe the same XY velocity.
        // A positive-only speed target fights legitimate reverse motion and
        // creates a nonzero steady-state position error near a reverse goal.
        Vec2 tangent = curve[index+1].head<2>()-curve[index].head<2>();
        if (tangent.norm() > 1e-10) tangent.normalize();
        const Vec2 direction(std::cos(cur_vel_array(1,t)),std::sin(cur_vel_array(1,t)));
        const double alignment = tangent.dot(direction);
        const double sign = alignment < -.05 ? -1.0 : alignment > .05 ? 1.0 :
                            (cur_vel_array(0,t) < 0.0 ? -1.0 : 1.0);
        out.ref_us(t) *= sign;
      }
    }
    return out;
  }

  Vec gear = Vec::Constant(T_, curve[point_index_](3));
  const double ref_speed_forward = ref_speed * dt_;

  for (int t = 0; t < T_; ++t) {
    pre_state = motionPredict(pre_state, cur_vel_array.col(t));
    out.nom_s.col(t + 1) = pre_state;

    if (ref_speed_forward >= interval_) {
      const int inc = static_cast<int>(ref_speed_forward / interval_);
      ref_index += inc;
      if (ref_index > n - 1) {
        ref_index = n - 1;
        gear(t) = 0.0;
      }
      ref_state = curve[ref_index].head<3>();
    } else {
      ref_state = findInteractionPoint(ref_state, ref_index,
                                       ref_speed_forward);
      if (ref_index > n - 1) gear(t) = 0.0;
    }

    ref_state(2) = pre_state(2) + wrapToPi(ref_state(2) - pre_state(2));
    out.ref_s.col(t + 1) = ref_state;
  }

  out.ref_us = gear * ref_speed;
  return out;
}

}  // namespace neupan
