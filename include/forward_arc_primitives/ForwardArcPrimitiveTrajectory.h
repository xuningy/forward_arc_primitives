/*
ForwardArcPrimitiveTrajectory
Copyright (C) 2019 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <tuple>
#include <deque>
#include <vector>

#include <ros/ros.h>
#include <forward_arc_primitives/ForwardArcMotionPrimitives.h>

namespace planner {

namespace forward_arc_primitive_trajectory {

    using Input = Eigen::Vector4d;  // Eigen::Vector4d(linear vel, angular vel, z vel, duration)
    using TrajectoryParam = std::tuple<Input, float>; // std::tuple<Input, duration>
    using Sequence = std::deque<TrajectoryParam>;
    using Trajectory = std::deque<ForwardArcMotionPrimitives>;

    // Methods for generating trajectories (ForwardArcMotionPrimitive objects)
    // NOTE: Usually a good idea to set yaw_rampup_duration to 0.3, and
    // aligned_heading to true.

    ForwardArcMotionPrimitives generatePrimitive(const TrajectoryParam &param, const FlatState& ref_state, float yaw_rampup_duration, bool aligned_heading);

    Trajectory constructTrajectory(const Sequence& sequence, const FlatState& ref_state, float yaw_rampup_duration, bool aligned_heading, FlatState* end_state);

    Trajectory constructTrajectory(const Sequence& sequence, const FlatState& ref_state, float yaw_rampup_duration, bool aligned_heading);

    void printSequence(const Sequence& sequence);
    void printReverseSequence(const Sequence& sequence);

    std::vector<FlatState> samplePath(const Trajectory& trajectory, float dt);

}// namespace forward_arc_primitive_trajectory

} // namespace planner
