/**
 * Pose Solver utilities.
 *
 * Giorgio Manca <giorgio.manca.97@gmail.com>
 *
 * August 6, 2025
 */

/**
 * Copyright 2025 dotX Automation s.r.l.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <pose_solver/pose_solver.hpp>

namespace pose_solver
{

double PoseSolver::wrap_angle(double angle) {
  if(angle >= 0) {
    return std::fmod(angle + M_PI, 2*M_PI) - M_PI;
  } else {
    return M_PI - std::fmod(M_PI - angle, 2*M_PI);
  }
}


Quaterniond PoseSolver::tri_attitude(const Vector3d &v1, const Vector3d &v2, const Vector3d &v3)
{
  Vector3d x = (v1 - ((v2 + v3) / 2.0)).normalized();
  Vector3d z = ((v2 - v1).cross(v3 - v1)).normalized();
  Vector3d y = (z.cross(x)).normalized();

  Matrix3d R;
  R << x.x(), y.x(), z.x(),
       x.y(), y.y(), z.y(),
       x.z(), y.z(), z.z();

  return Quaterniond(R);
}


PoseSolver::Solution PoseSolver::solve_pose(
  const std::vector<PositionIso> & positions,
  const Attitude & attitude)
{
  Solution sol;

  // Attitude computations

  if (positions.size() == 2ul) {
    double yaw_sensor = std::atan2(
      positions.at(0).position.point.y() - positions.at(1).position.point.y(),
      positions.at(0).position.point.x() - positions.at(1).position.point.x());

    double yaw_offset = std::atan2(
      positions.at(0).isometry.translation().y() - positions.at(1).isometry.translation().y(),
      positions.at(0).isometry.translation().x() - positions.at(1).isometry.translation().x());

    double yaw = wrap_angle(yaw_sensor - yaw_offset);

    if (attitude_completion_active()) {
      Vector3d eul = attitude.quat.toRotationMatrix().eulerAngles(2, 1, 0);
      AngleAxisd ax = AngleAxisd(eul.x(), Vector3d::UnitX());
      AngleAxisd ay = AngleAxisd(eul.y(), Vector3d::UnitY());
      AngleAxisd az = AngleAxisd(yaw, Vector3d::UnitZ());

      sol.attitude.quat = az * ay * ax;
      sol.attitude.cov(0,0) = attitude.cov(0,0);
      sol.attitude.cov(0,1) = attitude.cov(0,1);
      sol.attitude.cov(1,0) = attitude.cov(1,0);
      sol.attitude.cov(1,1) = attitude.cov(1,1);
      sol.attitude.cov(2,2) = attitude_covariance_(2,2);

    } else {
      sol.attitude.quat = AngleAxisd(yaw, Vector3d::UnitZ());
      sol.attitude.cov(2,2) = attitude_covariance_(2,2);
    }
  }
  else if (positions.size() == 3ul) {
    Quaterniond att_sensor = tri_attitude(
      positions.at(0).position.point,
      positions.at(1).position.point,
      positions.at(2).position.point);

    Quaterniond att_offset = tri_attitude(
      positions.at(0).isometry.translation(),
      positions.at(1).isometry.translation(),
      positions.at(2).isometry.translation());

    sol.attitude.quat = att_sensor * att_offset.inverse();
    sol.attitude.cov = attitude_covariance_;
  }
  else if (attitude_completion_active()) {
    sol.attitude.quat = attitude.quat;
    sol.attitude.cov = attitude.cov;
  } else {
    sol.attitude.quat = Quaterniond::Identity();
    sol.attitude.cov = attitude_covariance_;
  }

  // Position computations

  sol.position.point = Vector3d::Zero();
  sol.position.cov = Matrix3d::Zero();

  for (size_t i = 0ul; i < positions.size(); i++) {
    Vector3d trasl = positions.at(i).isometry.translation();
    sol.position.point += positions.at(i).position.point - sol.attitude.quat * trasl;
    sol.position.cov += positions.at(i).position.cov;
  }

  sol.position.point /= (double) positions.size();
  sol.position.cov /= (double) positions.size();

  return sol;
}

} // namespace pose_solver
