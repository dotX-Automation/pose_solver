/**
 * Pose Solver node constants.
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

/* Publishers topics. */
const std::string PoseSolver::pose_pub_topic_ = "~/pose";

/* Subscriptions topics. */
const std::string PoseSolver::aux_sub_topic_ = "/attitude";

/* Service client Names */
const std::string PoseSolver::get_transform_client_name_ = "/dua_tf_server/get_transform";

} // namespace pose_solver
