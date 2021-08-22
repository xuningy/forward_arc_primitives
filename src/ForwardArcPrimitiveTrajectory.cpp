
/*
ForwardArcPrimitiveTrajectory
Copyright (C) 2019 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <forward_arc_primitives/ForwardArcPrimitiveTrajectory.h>

namespace planner {

namespace forward_arc_primitive_trajectory {

ForwardArcMotionPrimitives generatePrimitive(const TrajectoryParam &param, const FlatState& start_state, float yaw_rampup_duration, bool aligned_heading)
{
  auto input = std::get<0>(param);
  auto duration = std::get<1>(param);

  return ForwardArcMotionPrimitives(start_state, input, duration, yaw_rampup_duration, aligned_heading);
}


Trajectory constructTrajectory(const Sequence& sequence, const FlatState& ref_state, float yaw_rampup_duration, bool aligned_heading, FlatState* end_state)
{
  Trajectory trajectory;
  FlatState start_state = ref_state;

  for (size_t i = 0; i < sequence.size(); i++) {
    auto segment = sequence[i];
    auto input = std::get<0>(segment);
    auto duration = std::get<1>(segment);

    // If duration is less than 100ms, the segment will be IGNORED.
    if (duration >= 0.1) {
      auto primitive = ForwardArcMotionPrimitives(start_state, input, duration, yaw_rampup_duration, aligned_heading);
      trajectory.push_back(primitive);

      // update the start state of next primitive to be the end of this trajectory
      start_state = primitive.getWorldPoseAtTime(primitive.tfinal());
    }
  }

  *end_state = start_state;
  return trajectory;
}

Trajectory constructTrajectory(const Sequence& sequence, const FlatState& ref_state, float yaw_rampup_duration, bool aligned_heading)
{
  FlatState final_state;
  return constructTrajectory(sequence, ref_state, yaw_rampup_duration, aligned_heading, &final_state);
}

std::vector<FlatState> samplePath(const Trajectory& trajectory, float dt)
{
  std::vector<FlatState> path;
  for (auto & primitive : trajectory)
  {
    std::vector<FlatState> subpath = primitive.samplePath(dt);
    path.insert(path.end(), subpath.begin(), subpath.end());
  }

  // Modify the final point's time (should be first point + duration). This is
  // a hack based on how the Waypoints constructor is determining the duration
  // of the trajectory.
  path.back().t = dt * (path.size() - 1) + path.front().t;

  return path;
}

void printSequence(const Sequence& sequence)
{
  // Sequence is of tuple(Eigen::Vector4d(v_x, omega, z, side), float T):
  std::cout << "start ";
  for (auto &segment : sequence)
  {
    auto input = std::get<0>(segment);
    auto duration = std::get<1>(segment);

    std::cout << "->({" << input(0) <<", "<< input(1) <<", " << input(2)<<", " << input(3)<<"}, "<< duration<<")";
  }
  std::cout << std::endl;
}

void printReverseSequence(const Sequence& sequence)
{
  // Sequence is of tuple(Eigen::Vector4d(v_x, omega, z, side), float T):
  for (auto &segment : sequence)
  {
    auto input = std::get<0>(segment);
    auto duration = std::get<1>(segment);

    std::cout << "({" << input(0) <<", "<< input(1) <<", " << input(2)<<", " << input(3)<<"}, "<< duration<<")->";
  }
  std::cout << " start " << std::endl;
}

}

} // namespace planner
