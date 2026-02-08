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

bool PoseSolver::get_isometry(
  const std::string & source, const std::string & target,
  const rclcpp::Time & time, Isometry3d & isometry)
{
  Time stamp = tf_ignore_stamp_ ? rclcpp::Time() : time;
  Header source_hdr{}, target_hdr{};

  source_hdr.frame_id = source;
  source_hdr.stamp = stamp;
  target_hdr.frame_id = target;
  target_hdr.stamp = stamp;
  auto timeout = rclcpp::Duration(std::chrono::nanoseconds(1000000 * tf_timeout_ms_));
  TransformStamped tf{};

  uint8_t res = get_transform(source_hdr, target_hdr, tf, true, timeout);

  if (res == CommandResultStamped::TIMEOUT || res == CommandResultStamped::ERROR) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
      "Error requesting transform from %s to %s",
      source.c_str(), target.c_str());
    return false;
  }

  Vector3d vect = Vector3d(
    tf.transform.translation.x,
    tf.transform.translation.y,
    tf.transform.translation.z);

  Quaterniond quat = Quaterniond(
    tf.transform.rotation.w,
    tf.transform.rotation.x,
    tf.transform.rotation.y,
    tf.transform.rotation.z);

  isometry.translation() = vect;
  isometry.linear() = quat.toRotationMatrix();

  return true;
}

} // namespace pose_solver
