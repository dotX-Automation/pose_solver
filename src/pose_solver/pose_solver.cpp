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

#include <pose_solver/pose_solver.hpp>

namespace pose_solver
{

PoseSolver::PoseSolver(const rclcpp::NodeOptions & opts)
: NodeBase("pose_solver", opts, true)
{
  dua_init_node();

  init_internals();

  RCLCPP_INFO(this->get_logger(), "Node initialized");
}

PoseSolver::~PoseSolver()
{}


void PoseSolver::init_cgroups()
{
  if (attitude_completion_active()) {
    aux_sub_cgroup_ = dua_create_exclusive_cgroup();
  }

  poses_sub_cgroup_.reserve(poses_topic_and_link_.size());
  for (size_t i = 0ul; i < poses_topic_and_link_.size(); i++) {
    poses_sub_cgroup_.push_back(dua_create_exclusive_cgroup());
  }
}


void PoseSolver::init_subscribers()
{
  if (attitude_completion_active() || poses_topic_and_link_.size() > 1ul) {
    poses_sub_.reserve(poses_topic_and_link_.size());
    for (size_t i = 0ul; i < poses_topic_and_link_.size(); i++) {
      rclcpp::SubscriptionOptions sub_opts;
      sub_opts.callback_group = poses_sub_cgroup_.at(i);

      auto pose_sub = std::make_shared<message_filters::Subscriber<PoseWithCovarianceStamped>>();
      pose_sub->subscribe(
        this,
        poses_topic_and_link_.at(i).topic,
        dua_qos::Reliable::get_datum_qos().get_rmw_qos_profile(),
        sub_opts);
      RCLCPP_INFO(
        get_logger(), "[TOPIC SUB] '%s'",
        pose_sub->getSubscriber()->get_topic_name());

      poses_sub_.push_back(pose_sub);
    }
  }

  if (attitude_completion_active()) {
    rclcpp::SubscriptionOptions sub_opts;
    sub_opts.callback_group = aux_sub_cgroup_;

    if (attitude_completion_source_type_ == AttitudeSource::Imu) {
      aux_imu_sub_ = std::make_shared<message_filters::Subscriber<Imu>>();
      aux_imu_sub_->subscribe(
        this,
        aux_sub_topic_,
        dua_qos::Reliable::get_datum_qos().get_rmw_qos_profile(),
        sub_opts);
      RCLCPP_INFO(get_logger(), "[TOPIC SUB] '%s'", aux_imu_sub_->getSubscriber()->get_topic_name());

      if (poses_topic_and_link_.size() == 1ul) {
        sync_1_imu_sub_ = std::make_shared<message_filters::Synchronizer<ApproxTime1Imu>>(
          ApproxTime1Imu(message_filters_queue_size_),
          *poses_sub_.at(0),
          *aux_imu_sub_);
        sync_1_imu_sub_->registerCallback(std::bind(
          &PoseSolver::input_1_imu_clbk,
          this,
          std::placeholders::_1,
          std::placeholders::_2));
      } else if (poses_topic_and_link_.size() == 2ul) {
        sync_2_imu_sub_ = std::make_shared<message_filters::Synchronizer<ApproxTime2Imu>>(
          ApproxTime2Imu(message_filters_queue_size_),
          *poses_sub_.at(0),
          *poses_sub_.at(1),
          *aux_imu_sub_);
        sync_2_imu_sub_->registerCallback(std::bind(
          &PoseSolver::input_2_imu_clbk,
          this,
          std::placeholders::_1,
          std::placeholders::_2,
          std::placeholders::_3));
      }
    }
    else if (attitude_completion_source_type_ == AttitudeSource::Odometry) {
      aux_odometry_sub_ = std::make_shared<message_filters::Subscriber<Odometry>>();
      aux_odometry_sub_->subscribe(
        this,
        aux_sub_topic_,
        dua_qos::Reliable::get_datum_qos().get_rmw_qos_profile(),
        sub_opts);
      RCLCPP_INFO(
        get_logger(), "[TOPIC SUB] '%s'",
        aux_odometry_sub_->getSubscriber()->get_topic_name());

      if (poses_topic_and_link_.size() == 1ul) {
        sync_1_odometry_sub_ = std::make_shared<message_filters::Synchronizer<ApproxTime1Odometry>>(
          ApproxTime1Odometry(message_filters_queue_size_),
          *poses_sub_.at(0),
          *aux_odometry_sub_);
        sync_1_odometry_sub_->registerCallback(std::bind(
          &PoseSolver::input_1_odometry_clbk,
          this,
          std::placeholders::_1,
          std::placeholders::_2));
      } else if (poses_topic_and_link_.size() == 2ul) {
        sync_2_odometry_sub_ = std::make_shared<message_filters::Synchronizer<ApproxTime2Odometry>>(
          ApproxTime2Odometry(message_filters_queue_size_),
          *poses_sub_.at(0),
          *poses_sub_.at(1),
          *aux_odometry_sub_);
        sync_2_odometry_sub_->registerCallback(std::bind(
          &PoseSolver::input_2_odometry_clbk,
          this,
          std::placeholders::_1,
          std::placeholders::_2,
          std::placeholders::_3));
      }
    }
  }
  else {
    if (poses_topic_and_link_.size() == 1ul) {
      single_sub_ = dua_create_subscription<PoseWithCovarianceStamped>(
        poses_topic_and_link_.at(0).topic,
        std::bind(
          &PoseSolver::input_1_clbk,
          this,
          std::placeholders::_1),
        dua_qos::Reliable::get_datum_qos(),
        poses_sub_cgroup_.at(0));
    } else if (poses_topic_and_link_.size() == 2ul) {
      sync_2_sub_ = std::make_shared<message_filters::Synchronizer<ApproxTime2>>(
        ApproxTime2(message_filters_queue_size_),
        *poses_sub_.at(0),
        *poses_sub_.at(1));
      sync_2_sub_->registerCallback(std::bind(
        &PoseSolver::input_2_clbk,
        this,
        std::placeholders::_1,
        std::placeholders::_2));
    } else if (poses_topic_and_link_.size() == 3ul) {
      sync_3_sub_ = std::make_shared<message_filters::Synchronizer<ApproxTime3>>(
        ApproxTime3(message_filters_queue_size_),
        *poses_sub_.at(0),
        *poses_sub_.at(1),
        *poses_sub_.at(2));
      sync_3_sub_->registerCallback(std::bind(
        &PoseSolver::input_3_clbk,
        this,
        std::placeholders::_1,
        std::placeholders::_2,
        std::placeholders::_3));
    }
  }
}


void PoseSolver::init_publishers()
{
  pose_pub_ = dua_create_publisher<PoseWithCovarianceStamped>(
    pose_pub_topic_,
    dua_qos::Reliable::get_datum_qos());
}


void PoseSolver::init_service_clients()
{
  get_transform_client_ = dua_create_service_client<GetTransform>(
    get_transform_client_name_);
}


void PoseSolver::init_internals()
{
  base_frame_ = tf_agent_prefix_ + tf_base_link_;
  aux_iso_ = Isometry3d::Identity();

  sensors_frame_.reserve(poses_topic_and_link_.size());
  poses_iso_.reserve(poses_topic_and_link_.size());
  sensors_iso_.reserve(poses_topic_and_link_.size());
  for (size_t i = 0ul; i < poses_topic_and_link_.size(); i++) {
    sensors_frame_.push_back(tf_agent_prefix_ + poses_topic_and_link_.at(i).link);
    poses_iso_.push_back(Isometry3d::Identity());
    sensors_iso_.push_back(Isometry3d::Identity());
  }

  if (attitude_completion_enable_ && !attitude_completion_active()) {
    std::string warn_msg = "Attitude completion disabled: ";
    if(attitude_completion_source_type_ == AttitudeSource::None) {
      warn_msg = warn_msg + "source type undefined";
    } else if (poses_topic_and_link_.size() >= 3ul) {
      warn_msg = warn_msg + "solution is already complete";
    }

    RCLCPP_WARN(this->get_logger(), warn_msg.c_str());
  }
}

} // namespace pose_solver

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(pose_solver::PoseSolver)
