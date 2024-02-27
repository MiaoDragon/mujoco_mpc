// Copyright 2022 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "mjpc/tasks/point_push/point_push.h"

#include <string>

#include <mujoco/mujoco.h>
#include "mjpc/utilities.h"

namespace mjpc {

std::string PointPush::XmlPath() const {
  return GetModelPath("point_push/task.xml");
}
std::string PointPush::Name() const { return "PointPush"; }

// -------- Residuals for point push task -------
//   Number of residuals: 3
//     Residual (0): velocity (controlled by velocity actuator)
//     Residual (1): obj_pos - goal_pos
//     Residual (2): obj_ori - goal_ori
// --------------------------------------------

void PointPush::ResidualFn::Residual(const mjModel* model, const mjData* data,
                                    double* residual) const {
  // initialize counter
  int counter = 0;

  // ----- residual (0-2) ----- //
  mju_copy(residual, data->ctrl, model->nu);  // ctrl is velocity
  counter += 3;


  // ---------- Residual (3-5) ----------
  // goal position
  double* goal_position = SensorByName(model, data, "goal_position");

  // system's position
  double* position = SensorByName(model, data, "obj_position");

  // position error
  mju_sub3(residual + counter, position, goal_position);
  counter += 3;

  // ---------- Residual (6-8) ----------
  // goal orientation
  double* goal_orientation = SensorByName(model, data, "goal_orientation");

  // system's orientation
  double* orientation = SensorByName(model, data, "obj_orientation");
  mju_normalize4(goal_orientation);

  // orientation error
  mju_subQuat(residual + counter, goal_orientation, orientation);
  counter += 3;
}

void PointPush::TransitionLocked(mjModel* model, mjData* data) {
//   // some Lissajous curve
//   double goal[2]{0.25 * mju_sin(data->time), 0.25 * mju_cos(data->time / mjPI)};

//   // update mocap position
//   data->mocap_pos[0] = goal[0];
//   data->mocap_pos[1] = goal[1];
}
}  // namespace mjpc
