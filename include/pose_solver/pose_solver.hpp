/**
 * Pose Solver node definition.
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

#pragma once

#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>

#include <atomic>
#include <bitset>
#include <chrono>
#include <exception>
#include <memory>
#include <mutex>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rmw/qos_profiles.h>

#include <dua_node_cpp/dua_node.hpp>
#include <dua_qos_cpp/dua_qos.hpp>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <dua_common_interfaces/msg/command_result_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>

using namespace Eigen;

using namespace builtin_interfaces::msg;
using namespace dua_common_interfaces::msg;
using namespace geometry_msgs::msg;
using namespace message_filters;
using namespace nav_msgs::msg;
using namespace rcl_interfaces::msg;
using namespace sensor_msgs::msg;
using namespace std_msgs::msg;

namespace pose_solver
{

typedef PoseWithCovarianceStamped PWCS;

typedef sync_policies::ApproximateTime<PWCS, Imu> ApproxTime1Imu;
typedef sync_policies::ApproximateTime<PWCS, Odometry> ApproxTime1Odometry;
typedef sync_policies::ApproximateTime<PWCS, PWCS> ApproxTime2;
typedef sync_policies::ApproximateTime<PWCS, PWCS, Imu> ApproxTime2Imu;
typedef sync_policies::ApproximateTime<PWCS, PWCS, Odometry> ApproxTime2Odometry;
typedef sync_policies::ApproximateTime<PWCS, PWCS, PWCS> ApproxTime3;


class PoseSolver : public dua_node::NodeBase
{
public:
  /**
   * @brief Builds a new PoseSolver node.
   *
   * @param opts Node options.
   *
   * @throws RuntimeError
   */
  PoseSolver(const rclcpp::NodeOptions & opts = rclcpp::NodeOptions());

  /**
   * @brief Finalizes node operation.
   */
  ~PoseSolver();

private:
  /* Node structures */
  struct Sensor {
    std::string topic;
    std::string link;
  };

  struct Position {
    Vector3d point;
    Matrix3d cov;

    Position() : point(Vector3d::Zero()) , cov(Matrix3d::Zero()) {}
  };

  struct Attitude {
    Quaterniond quat;
    Matrix3d cov;

    Attitude() : quat(Quaterniond::Identity()) , cov(Matrix3d::Zero()) {}
  };

  struct PositionMsg {
    Header header;
    Position position;

    PositionMsg() {}

    PositionMsg(const PoseWithCovarianceStamped & msg) {
      header = msg.header;
      position.point.x() = msg.pose.pose.position.x;
      position.point.y() = msg.pose.pose.position.y;
      position.point.z() = msg.pose.pose.position.z;
      position.cov(0,0) = msg.pose.covariance.at(0);
      position.cov(0,1) = msg.pose.covariance.at(1);
      position.cov(0,2) = msg.pose.covariance.at(2);
      position.cov(1,0) = msg.pose.covariance.at(6);
      position.cov(1,1) = msg.pose.covariance.at(7);
      position.cov(1,2) = msg.pose.covariance.at(8);
      position.cov(2,0) = msg.pose.covariance.at(12);
      position.cov(2,1) = msg.pose.covariance.at(13);
      position.cov(2,2) = msg.pose.covariance.at(14);
    }
  };

  struct AttitudeMsg {
    Header header;
    Attitude attitude;

    AttitudeMsg() {}

    AttitudeMsg(const Imu & msg) {
      header = msg.header;
      attitude.quat.w() = msg.orientation.w;
      attitude.quat.x() = msg.orientation.x;
      attitude.quat.y() = msg.orientation.y;
      attitude.quat.z() = msg.orientation.z;
      attitude.cov(0,0) = msg.orientation_covariance.at(0);
      attitude.cov(0,1) = msg.orientation_covariance.at(1);
      attitude.cov(0,2) = msg.orientation_covariance.at(2);
      attitude.cov(1,0) = msg.orientation_covariance.at(3);
      attitude.cov(1,1) = msg.orientation_covariance.at(4);
      attitude.cov(1,2) = msg.orientation_covariance.at(5);
      attitude.cov(2,0) = msg.orientation_covariance.at(6);
      attitude.cov(2,1) = msg.orientation_covariance.at(7);
      attitude.cov(2,2) = msg.orientation_covariance.at(8);
    }

    AttitudeMsg(const Odometry & msg) {
      header = msg.header;
      attitude.quat.w() = msg.pose.pose.orientation.w;
      attitude.quat.x() = msg.pose.pose.orientation.x;
      attitude.quat.y() = msg.pose.pose.orientation.y;
      attitude.quat.z() = msg.pose.pose.orientation.z;
      attitude.cov(0,0) = msg.pose.covariance.at(21);
      attitude.cov(0,1) = msg.pose.covariance.at(22);
      attitude.cov(0,2) = msg.pose.covariance.at(23);
      attitude.cov(1,0) = msg.pose.covariance.at(27);
      attitude.cov(1,1) = msg.pose.covariance.at(28);
      attitude.cov(1,2) = msg.pose.covariance.at(29);
      attitude.cov(2,0) = msg.pose.covariance.at(33);
      attitude.cov(2,1) = msg.pose.covariance.at(34);
      attitude.cov(2,2) = msg.pose.covariance.at(35);
    }
  };

  struct PositionIso {
    Position position;
    Isometry3d isometry;
  };


  struct Solution {
    Position position;
    Attitude attitude;
  };

  /* Node enumerators */
  enum class AttitudeSource : uint8_t
  {
    None = 0u,
    Imu = 1u,
    Odometry = 2u
  };

  /* Node initialization routines. */
  void init_parameters() override;
  void init_cgroups() override;
  void init_subscribers() override;
  void init_publishers() override;

  /**
   * @brief Routine to initialize node structures.
   */
  void init_internals();

  /* Node parameters. */
  bool attitude_completion_enable_ = false;
  AttitudeSource attitude_completion_source_type_ = AttitudeSource::None;
  Matrix3d attitude_covariance_ = Matrix3d::Zero();
  int64_t message_filters_queue_size_ = 0;
  std::vector<Sensor> poses_topic_and_link_ = {};
  std::string tf_agent_prefix_ = "";
  std::string tf_base_link_ = "";
  std::string tf_fixed_frame_ = "";
  bool tf_ignore_stamp_ = false;
  bool tf_static_sensors_ = false;
  int64_t tf_timeout_ms_ = 0;

  /* Node Parameters Validators. */
  bool validate_attitude_completion_source_type(const rclcpp::Parameter & p);
  bool validate_attitude_covariance(const rclcpp::Parameter & p);
  bool validate_poses_topic_and_link(const rclcpp::Parameter & p);

  /* Node variables. */
  std::string base_frame_;
  std::vector<std::string> sensors_frame_;
  Isometry3d aux_iso_;
  std::vector<Isometry3d> poses_iso_;
  std::vector<Isometry3d> sensors_iso_;
  bool tf_static_updated_ = false;

  /* Publishers. */
  rclcpp::Publisher<PoseWithCovarianceStamped>::SharedPtr pose_pub_;

  /* Publishers topics. */
  static const std::string pose_pub_topic_;

  /* Publishers routines. */
  /**
   * @brief Publish computed pose.
   *
   * @param timestamp Pose timestamp.
   * @param pose Pose to publish.
   */
  void publish(const Time & timestamp, const Solution & solution);

  /* Subscriptions. */
  rclcpp::Subscription<PoseWithCovarianceStamped>::SharedPtr single_sub_;

  std::shared_ptr<message_filters::Subscriber<Imu>> aux_imu_sub_;
  std::shared_ptr<message_filters::Subscriber<Odometry>> aux_odometry_sub_;
  std::vector<std::shared_ptr<message_filters::Subscriber<PoseWithCovarianceStamped>>> poses_sub_;

  std::shared_ptr<Synchronizer<ApproxTime1Imu>> sync_1_imu_sub_;
  std::shared_ptr<Synchronizer<ApproxTime1Odometry>> sync_1_odometry_sub_;
  std::shared_ptr<Synchronizer<ApproxTime2>> sync_2_sub_;
  std::shared_ptr<Synchronizer<ApproxTime2Imu>> sync_2_imu_sub_;
  std::shared_ptr<Synchronizer<ApproxTime2Odometry>> sync_2_odometry_sub_;
  std::shared_ptr<Synchronizer<ApproxTime3>> sync_3_sub_;

  /* Subscriptions topics. */
  static const std::string aux_sub_topic_;

  /* Subscriptions callback groups. */
  rclcpp::CallbackGroup::SharedPtr aux_sub_cgroup_;
  std::vector<rclcpp::CallbackGroup::SharedPtr> poses_sub_cgroup_;

  /* Subscriptions callbacks. */
  /**
   * @brief Callback to compute solution with one point.
   *
   * @param pose1 PoseWithCovarianceStamped message to parse.
   */
  void input_1_clbk(
    const PoseWithCovarianceStamped::ConstSharedPtr & pose1);

  /**
   * @brief Callback to compute solution with one point and attitude completion with imu.
   *
   * @param pose1 PoseWithCovarianceStamped message to parse.
   * @param aux Imu message to parse.
   */
  void input_1_imu_clbk(
    const PoseWithCovarianceStamped::ConstSharedPtr & pose1,
    const Imu::ConstSharedPtr & aux);

  /**
   * @brief Callback to compute solution with one point and attitude completion with odometry.
   *
   * @param pose1 PoseWithCovarianceStamped message to parse.
   * @param aux Odometry message to parse.
   */
  void input_1_odometry_clbk(
    const PoseWithCovarianceStamped::ConstSharedPtr & pose1,
    const Odometry::ConstSharedPtr & aux);

  /**
   * @brief Callback to compute solution with two points.
   *
   * @param pose1 PoseWithCovarianceStamped message to parse.
   * @param pose2 PoseWithCovarianceStamped message to parse.
   */
  void input_2_clbk(
    const PoseWithCovarianceStamped::ConstSharedPtr & pose1,
    const PoseWithCovarianceStamped::ConstSharedPtr & pose2);

  /**
   * @brief Callback to compute solution with two points and attitude completion with imu.
   *
   * @param pose1 PoseWithCovarianceStamped message to parse.
   * @param pose2 PoseWithCovarianceStamped message to parse.
   * @param aux Imu message to parse.
   */
  void input_2_imu_clbk(
    const PoseWithCovarianceStamped::ConstSharedPtr & pose1,
    const PoseWithCovarianceStamped::ConstSharedPtr & pose2,
    const Imu::ConstSharedPtr & aux);

  /**
   * @brief Callback to compute solution with two points and attitude completion with odometry.
   *
   * @param pose1 PoseWithCovarianceStamped message to parse.
   * @param pose2 PoseWithCovarianceStamped message to parse.
   * @param aux Imu message to parse.
   */
  void input_2_odometry_clbk(
    const PoseWithCovarianceStamped::ConstSharedPtr & pose1,
    const PoseWithCovarianceStamped::ConstSharedPtr & pose2,
    const Odometry::ConstSharedPtr & aux);

  /**
   * @brief Callback to compute solution with three points.
   *
   * @param pose1 PoseWithCovarianceStamped message to parse.
   * @param pose2 PoseWithCovarianceStamped message to parse.
   * @param pose3 PoseWithCovarianceStamped message to parse.
   */
  void input_3_clbk(
    const PoseWithCovarianceStamped::ConstSharedPtr & pose1,
    const PoseWithCovarianceStamped::ConstSharedPtr & pose2,
    const PoseWithCovarianceStamped::ConstSharedPtr & pose3);

  /**
   * @brief Callback to compute solution.
   *
   * @param positions Positions to parse.
   * @param attitude Attitude to parse.
   */
  void general_clbk(
    const std::vector<PositionMsg> & positions,
    const AttitudeMsg & attitude);

  /* Utility routines */

  /**
   * @brief Retrieve a transform as an Eigen::Isometry3d.
   *
   * @param source Source frame.
   * @param target Target frame.
   * @param time Request timestamp.
   * @param transform Transform result.
   *
   * @return True if the transformation is available, false otherwise.
   */
  bool get_isometry(
    const std::string & source, const std::string & target,
    const rclcpp::Time & time, Isometry3d & isometry);

  /**
   * @brief Determine if the attitude completion is active.
   */
  inline bool attitude_completion_active() {
    return attitude_completion_enable_
      && attitude_completion_source_type_ != AttitudeSource::None
      && poses_topic_and_link_.size() < 3ul;
  }

  /**
   * @brief Wrap angle between -pi and pi.
   *
   * @param angle Angle in radians.
   */
  double wrap_angle(double angle);

  /**
   * @brief Compute the attidude of a triangle.
   *
   * @param v1 Front vertex.
   * @param v2 Rear left vertex.
   * @param v3 Rear right vertex.
   */
  Quaterniond tri_attitude(const Vector3d &v1, const Vector3d &v2, const Vector3d &v3);

  /**
   * @brief Compute agent point using the known position of multiple sensors.
   *
   * @param points Sensor points with their respective isometry.
   * @param attitude Agent known attitude for attitude completion.
   */
  Solution solve_pose(
    const std::vector<PositionIso> & positions,
    const Attitude & attitude);
};

} // namespace pose_solver
