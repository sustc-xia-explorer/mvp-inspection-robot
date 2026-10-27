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

#include "neupan/robot.hpp"

#include <cmath>
#include <algorithm>
#include <limits>
#include <stdexcept>

namespace neupan {

namespace {

// Cross product sign sweep: returns +1 for CCW, -1 for CW, throws if the
// polygon is not convex (mirrors neupan.util.is_convex_and_ordered).
int convexOrientation(const Mat2X& v) {
  const Eigen::Index n = v.cols();
  if (n < 3) throw std::invalid_argument("robot: polygon needs >= 3 vertices");

  double direction = 0.0;
  for (Eigen::Index i = 0; i < n; ++i) {
    const Vec2 o = v.col(i);
    const Vec2 a = v.col((i + 1) % n);
    const Vec2 b = v.col((i + 2) % n);
    const double cross =
        (a.x() - o.x()) * (b.y() - o.y()) - (a.y() - o.y()) * (b.x() - o.x());
    if (cross != 0.0) {
      if (direction == 0.0) {
        direction = cross;
      } else if (direction * cross < 0.0) {
        throw std::invalid_argument("robot: polygon is not convex");
      }
    }
  }
  return direction > 0.0 ? 1 : -1;
}

}  // namespace

void genInequalFromVertex(const Mat2X& vertices, Mat& G, Vec& h) {
  Mat2X v = vertices;
  if (convexOrientation(v) < 0) {
    // CW input: keep first vertex, reverse the rest (as upstream does).
    Mat2X r(2, v.cols());
    r.col(0) = v.col(0);
    for (Eigen::Index i = 1; i < v.cols(); ++i)
      r.col(i) = v.col(v.cols() - i);
    v = r;
  }

  const Eigen::Index num = v.cols();
  G.resize(num, 2);
  h.resize(num);

  for (Eigen::Index i = 0; i < num; ++i) {
    const Vec2 pre = v.col(i);
    const Vec2 next = v.col((i + 1) % num);
    const Vec2 diff = next - pre;
    G(i, 0) = diff.y();
    G(i, 1) = -diff.x();
    h(i) = G(i, 0) * pre.x() + G(i, 1) * pre.y();
  }
}

Robot::Robot(Kinematics kinematics, int receding, double step_time,
             Vec2 max_speed_in, Vec2 max_acce, const Mat2X& vertices_in)
    : kinematics(kinematics),
      T(receding),
      dt(step_time),
      max_speed(std::move(max_speed_in)),
      acce_bound(max_acce * step_time),
      vertices(vertices_in) {
  genInequalFromVertex(vertices, G, h);
}

Robot Robot::diffRectangle(int receding, double step_time, Vec2 max_speed,
                           Vec2 max_acce, double length, double width,
                           double wheelbase) {
  return rectangle(Kinematics::Diff, receding, step_time, max_speed, max_acce,
                   length, width, wheelbase);
}

Robot Robot::rectangle(Kinematics model, int receding, double step_time,
                       Vec2 max_speed, Vec2 max_acce, double length,
                       double width, double wheelbase) {
  const double sx = -(length - wheelbase) / 2.0;
  const double sy = -width / 2.0;
  Mat2X v(2, 4);
  v.col(0) << sx, sy;
  v.col(1) << sx + length, sy;
  v.col(2) << sx + length, sy + width;
  v.col(3) << sx, sy + width;
  return Robot(model, receding, step_time, std::move(max_speed),
               std::move(max_acce), v);
}

void Robot::linearize(const Vec3& nom_s_t, const Vec2& nom_u_t, Mat33& A,
                      Mat32& B, Vec3& C) const {
  const double phi = nom_s_t(2);
  const double v = nom_u_t(0);

  switch (kinematics) {
    case Kinematics::Diff:
      A << 1, 0, -v * dt * std::sin(phi),  //
          0, 1, v * dt * std::cos(phi),    //
          0, 0, 1;
      B << std::cos(phi) * dt, 0,  //
          std::sin(phi) * dt, 0,   //
          0, dt;
      C << phi * v * std::sin(phi) * dt, -phi * v * std::cos(phi) * dt, 0;
      break;
    case Kinematics::Omni: {
      const double direction = nom_u_t(1);
      A.setIdentity();
      B << std::cos(direction) * dt, -v * std::sin(direction) * dt,
           std::sin(direction) * dt,  v * std::cos(direction) * dt,
           0, 0;
      C << direction * v * std::sin(direction) * dt,
          -direction * v * std::cos(direction) * dt, 0;
      break;
    }
  }
}

const char* kinematicsName(Kinematics model) {
  return model == Kinematics::Omni ? "omni" : "diff";
}

Kinematics parseKinematics(const std::string& model) {
  if (model == "diff") return Kinematics::Diff;
  if (model == "omni") return Kinematics::Omni;
  throw std::invalid_argument("robot.kinematics must be 'diff' or 'omni': " + model);
}

Vec2 worldToBody(const Vec2& v, double yaw) {
  return Vec2(std::cos(yaw)*v.x() + std::sin(yaw)*v.y(),
             -std::sin(yaw)*v.x() + std::cos(yaw)*v.y());
}

Vec2 bodyToWorld(const Vec2& v, double yaw) {
  return Vec2(std::cos(yaw)*v.x() - std::sin(yaw)*v.y(),
              std::sin(yaw)*v.x() + std::cos(yaw)*v.y());
}

Vec3 Robot::predict(const Vec3& state, const Vec2& u) const {
  Vec3 next = state;
  const double direction = kinematics == Kinematics::Omni ? u(1) : state(2);
  next(0) += dt * u(0) * std::cos(direction);
  next(1) += dt * u(0) * std::sin(direction);
  if (kinematics == Kinematics::Diff) next(2) += dt * u(1);
  return next;
}

Vec2 Robot::controlToAction(const Vec2& u) const {
  if (kinematics == Kinematics::Diff) return u;
  return Vec2(u(0)*std::cos(u(1)), u(0)*std::sin(u(1)));
}

Vec2 Robot::stoppingControl(const Vec2& previous) const {
  return kinematics == Kinematics::Omni ? Vec2(0, previous(1)) : Vec2::Zero();
}

Vec2 Robot::actionToControl(const Vec2& action, const Vec2& reference) const {
  if (!action.allFinite()) throw std::invalid_argument("non-finite action");
  if (kinematics == Kinematics::Diff) return action;
  const double speed = action.norm();
  if (speed < 1e-10) return stoppingControl(reference);
  const double pi = std::acos(-1.0), direction = std::atan2(action.y(), action.x());
  Vec2 best = Vec2::Zero();
  double cost = std::numeric_limits<double>::infinity();
  // Equivalent signed-speed representations avoid a false 2*pi steering jump
  // when actual Cartesian execution is fed back to the polar optimizer.
  for (int sign : {1, -1}) {
    const double angle = direction + (sign < 0 ? pi : 0.0);
    const double center = std::round((reference(1)-angle)/(2*pi));
    for (int offset = -1; offset <= 1; ++offset) {
      const double candidate_angle = angle + (center+offset)*2*pi;
      if (std::abs(candidate_angle) > max_speed(1)+1e-6) continue;
      const Vec2 candidate(sign*speed, candidate_angle);
      const double candidate_cost =
          (candidate-reference).cwiseQuotient(acce_bound).squaredNorm();
      if (candidate_cost < cost) { best = candidate; cost = candidate_cost; }
    }
  }
  if (!std::isfinite(cost))
    throw std::invalid_argument("applied direction outside configured omni angle range");
  best(1) = std::clamp(best(1), -max_speed(1), max_speed(1));
  return best;
}

}  // namespace neupan
