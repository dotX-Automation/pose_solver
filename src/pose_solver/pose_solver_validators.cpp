/**
 * Pose Solver validators.
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


bool PoseSolver::validate_attitude_completion_source_type(const rclcpp::Parameter & p)
{
  std::string type = p.as_string();

  if (type == "imu") {
    attitude_completion_source_type_ = AttitudeSource::Imu;
  } else if (type == "odometry") {
    attitude_completion_source_type_ = AttitudeSource::Odometry;
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "PoseSolver::validate_attitude_completion_source_type: Invalid attitude.completion.source.type parameter: %s",
      type.c_str());
    return false;
  }

  return true;
}


bool PoseSolver::validate_attitude_covariance(const rclcpp::Parameter & p)
{
  std::vector<double> cov = p.as_double_array();

  if (cov.size() == 3ul) {
    attitude_covariance_(0,0) = cov.at(0);
    attitude_covariance_(1,1) = cov.at(1);
    attitude_covariance_(2,2) = cov.at(2);
  } else if (cov.size() == 9ul) {
    attitude_covariance_(0,0) = cov.at(0);
    attitude_covariance_(0,1) = cov.at(1);
    attitude_covariance_(0,2) = cov.at(2);
    attitude_covariance_(1,0) = cov.at(3);
    attitude_covariance_(1,1) = cov.at(4);
    attitude_covariance_(1,2) = cov.at(5);
    attitude_covariance_(2,0) = cov.at(6);
    attitude_covariance_(2,1) = cov.at(7);
    attitude_covariance_(2,2) = cov.at(8);
  } else {
    RCLCPP_ERROR(
      get_logger(),
      "PoseSolver::validate_attitude_covariance: Invalid attitude.covariance size");
    return false;
  }

  return true;
}


bool PoseSolver::validate_poses_topic_and_link(const rclcpp::Parameter & p)
{
  std::vector<std::string> list = p.as_string_array();

  if (list.size() < 1ul || list.size() > 3ul) {
    RCLCPP_ERROR(
      get_logger(),
      "PoseSolver::validate_poses_topic_and_link: Invalid poses_topic_and_link size");
    return false;
  }

  poses_topic_and_link_.clear();
  poses_topic_and_link_.reserve(list.size());
  for (const std::string & topic_and_link : list) {
    size_t pos = topic_and_link.find("@");
    if (pos == std::string::npos || pos == 0ul || pos == topic_and_link.size() - 1ul) {
      RCLCPP_ERROR(
        get_logger(),
        "PoseSolver::validate_poses_topic_and_link: Invalid poses_topic_and_link element: %s",
        topic_and_link.c_str());
      return false;
    } else {
      Sensor sensor;
      sensor.topic = topic_and_link.substr(0ul, pos);
      sensor.link = topic_and_link.substr(pos + 1ul);
      poses_topic_and_link_.push_back(sensor);
    }
  }

  return true;
}

} // namespace pose_solver
