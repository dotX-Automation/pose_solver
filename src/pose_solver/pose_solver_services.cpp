/**
 * Pose Solver services routine.
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

bool PoseSolver::get_transform(
  const std::string & source, const std::string & target,
  const rclcpp::Time & time, Isometry3d & isometry)
{
  Time stamp = tf_ignore_stamp_ ? rclcpp::Time() : time;

  auto req = std::make_shared<GetTransform::Request>();
  req->source.frame_id = target;
  req->source.stamp = stamp;
  req->target.frame_id = source;
  req->target.stamp = stamp;
  req->timeout = rclcpp::Duration(std::chrono::nanoseconds(1000 * tf_timeout_ms_));

  auto resp = get_transform_client_->call_sync(req);
  if (resp == nullptr || resp->result.result != CommandResultStamped::SUCCESS) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
      "Error requesting transform from %s to %s",
      source.c_str(), target.c_str());
    return false;
  }

  Vector3d vect = Vector3d(
    resp->transform.transform.translation.x,
    resp->transform.transform.translation.y,
    resp->transform.transform.translation.z);

  Quaterniond quat = Quaterniond(
    resp->transform.transform.rotation.w,
    resp->transform.transform.rotation.x,
    resp->transform.transform.rotation.y,
    resp->transform.transform.rotation.z);

  isometry.translation() = vect;
  isometry.linear() = quat.toRotationMatrix();

  return true;
}

} // namespace pose_solver
