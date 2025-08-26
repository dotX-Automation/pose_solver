/**
 * Pose Solver subscriptions ruotine.
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

void PoseSolver::input_1_clbk(
  const PoseWithCovarianceStamped::ConstSharedPtr & pose1)
{
  AttitudeMsg attitude;
  attitude.header.frame_id = tf_fixed_frame_;
  attitude.header.stamp = rclcpp::Time();

  std::vector<PositionMsg> positions;
  positions.reserve(1);
  positions.push_back(PositionMsg(*pose1));

  general_clbk(positions, attitude);
}


void PoseSolver::input_1_imu_clbk(
  const PoseWithCovarianceStamped::ConstSharedPtr & pose1,
  const Imu::ConstSharedPtr & aux)
{
  AttitudeMsg attitude(*aux);

  std::vector<PositionMsg> positions;
  positions.reserve(1);
  positions.push_back(PositionMsg(*pose1));
  
  general_clbk(positions, attitude);
}


void PoseSolver::input_1_odometry_clbk(
  const PoseWithCovarianceStamped::ConstSharedPtr & pose1,
  const Odometry::ConstSharedPtr & aux)
{
  AttitudeMsg attitude(*aux);

  std::vector<PositionMsg> positions;
  positions.reserve(1);
  positions.push_back(PositionMsg(*pose1));

  general_clbk(positions, attitude);
}


void PoseSolver::input_2_clbk(
  const PoseWithCovarianceStamped::ConstSharedPtr & pose1,
  const PoseWithCovarianceStamped::ConstSharedPtr & pose2)
{
  AttitudeMsg attitude;
  attitude.header.frame_id = tf_fixed_frame_;
  attitude.header.stamp = rclcpp::Time();

  std::vector<PositionMsg> positions;
  positions.reserve(2);
  positions.push_back(PositionMsg(*pose1));
  positions.push_back(PositionMsg(*pose2));
  
  general_clbk(positions, attitude);
}


void PoseSolver::input_2_imu_clbk(
  const PoseWithCovarianceStamped::ConstSharedPtr & pose1,
  const PoseWithCovarianceStamped::ConstSharedPtr & pose2,
  const Imu::ConstSharedPtr & aux)
{
  AttitudeMsg attitude(*aux);

  std::vector<PositionMsg> positions;
  positions.reserve(2);
  positions.push_back(PositionMsg(*pose1));
  positions.push_back(PositionMsg(*pose2));

  general_clbk(positions, attitude);
}


void PoseSolver::input_2_odometry_clbk(
  const PoseWithCovarianceStamped::ConstSharedPtr & pose1,
  const PoseWithCovarianceStamped::ConstSharedPtr & pose2,
  const Odometry::ConstSharedPtr & aux)
{
  AttitudeMsg attitude(*aux);

  std::vector<PositionMsg> positions;
  positions.reserve(2);
  positions.push_back(PositionMsg(*pose1));
  positions.push_back(PositionMsg(*pose2));

  general_clbk(positions, attitude);
}


void PoseSolver::input_3_clbk(
  const PoseWithCovarianceStamped::ConstSharedPtr & pose1,
  const PoseWithCovarianceStamped::ConstSharedPtr & pose2,
  const PoseWithCovarianceStamped::ConstSharedPtr & pose3)
{
  AttitudeMsg attitude;
  attitude.header.frame_id = tf_fixed_frame_;
  attitude.header.stamp = rclcpp::Time();

  std::vector<PositionMsg> positions;
  positions.reserve(3);
  positions.push_back(PositionMsg(*pose1));
  positions.push_back(PositionMsg(*pose2));
  positions.push_back(PositionMsg(*pose3));
  
  general_clbk(positions, attitude);
}


void PoseSolver::general_clbk(
    const std::vector<PositionMsg> & positions,
    const AttitudeMsg & attitude)
{
  rclcpp::Time timestamp = attitude.header.stamp;

  for (size_t i = 0ul; i < positions.size(); i++) {
    if (positions.at(i).header.stamp.sec > timestamp.seconds() ||
      (positions.at(i).header.stamp.sec == timestamp.seconds() && 
      positions.at(i).header.stamp.nanosec > timestamp.nanoseconds()))
    {
      timestamp = positions.at(i).header.stamp;
    }
  }

  std::vector<PositionIso> arg_positions_iso;
  arg_positions_iso.reserve(positions.size());

  for (size_t i = 0ul; i < positions.size(); i++) {
    PositionIso pos_iso;

    if (tf_static_sensors_) {
      pos_iso.isometry = sensors_iso_.at(i);
    } else {
      bool res = get_transform(
        base_frame_,
        sensors_frame_.at(i),
        positions.at(i).header.stamp,
        pos_iso.isometry);
      if (!res) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, 
          "Cannot retrive body-sensor TF for sensor %ld.", i);
        return;
      }
    }

    if (positions.at(i).header.frame_id == tf_fixed_frame_) {
      pos_iso.position = positions.at(i).position;
    } else {
      Isometry3d iso;
      bool res = get_transform(
        tf_fixed_frame_,
        positions.at(i).header.frame_id,
        positions.at(i).header.stamp,
        iso);
      if (!res) {
        RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, 
          "Cannot retrive fixed frames TF for sensor %ld.", i);
        return;
      } else {
        pos_iso.position.point = iso * positions.at(i).position.point;
        pos_iso.position.cov = iso.linear() * pos_iso.position.cov * iso.linear().transpose();
      }
    }

    arg_positions_iso.push_back(pos_iso);
  }
  
  Attitude arg_attitude;

  if (attitude.header.frame_id == tf_fixed_frame_) {
    arg_attitude = attitude.attitude;
  } else {
    Isometry3d iso;
    bool res = get_transform(
      tf_fixed_frame_,
      attitude.header.frame_id,
      attitude.header.stamp,
      iso);
    if (!res) {
      RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000, 
        "Cannot retrive fixed frames TF for attitude completition");
      return;
    } else {
      arg_attitude.quat = iso.linear() * attitude.attitude.quat;
      arg_attitude.cov = iso.linear() * attitude.attitude.cov * iso.linear().transpose();
    }
  }

  Solution sol = solve_pose(arg_positions_iso, arg_attitude);

  publish(timestamp, sol);
}

} // namespace pose_solver
