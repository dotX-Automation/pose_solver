/**
 * Pose Solver publishers routine.
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

void PoseSolver::publish(const Time & timestamp, const Solution & solution)
{
  PoseWithCovarianceStamped msg;

  msg.header.frame_id = tf_fixed_frame_;
  msg.header.stamp = timestamp;
  msg.pose.pose.position.x = solution.position.point.x();
  msg.pose.pose.position.y = solution.position.point.y();
  msg.pose.pose.position.z = solution.position.point.z();
  msg.pose.pose.orientation.w = solution.attitude.quat.w();
  msg.pose.pose.orientation.x = solution.attitude.quat.x();
  msg.pose.pose.orientation.y = solution.attitude.quat.y();
  msg.pose.pose.orientation.z = solution.attitude.quat.z();
  msg.pose.covariance.at(0) = solution.position.cov(0,0);
  msg.pose.covariance.at(1) = solution.position.cov(0,1);
  msg.pose.covariance.at(2) = solution.position.cov(0,2);
  msg.pose.covariance.at(6) = solution.position.cov(1,0);
  msg.pose.covariance.at(7) = solution.position.cov(1,1);
  msg.pose.covariance.at(8) = solution.position.cov(1,2);
  msg.pose.covariance.at(12) = solution.position.cov(2,0);
  msg.pose.covariance.at(13) = solution.position.cov(2,1);
  msg.pose.covariance.at(14) = solution.position.cov(2,2);
  msg.pose.covariance.at(21) = solution.attitude.cov(0,0);
  msg.pose.covariance.at(22) = solution.attitude.cov(0,1);
  msg.pose.covariance.at(23) = solution.attitude.cov(0,2);
  msg.pose.covariance.at(27) = solution.attitude.cov(1,0);
  msg.pose.covariance.at(28) = solution.attitude.cov(1,1);
  msg.pose.covariance.at(29) = solution.attitude.cov(1,2);
  msg.pose.covariance.at(33) = solution.attitude.cov(2,0);
  msg.pose.covariance.at(34) = solution.attitude.cov(2,1);
  msg.pose.covariance.at(35) = solution.attitude.cov(2,2);

  pose_pub_->publish(msg);
}

} // namespace pose_solver
