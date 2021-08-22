/*
Cost.h
Copyright (C) 2019 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once
#include <Eigen/Geometry>

#include <cpp_utils/linalg_utils.h>
#include <planning_representations/FlatState.h>

#include <forward_arc_primitives/ForwardArcMotionPrimitives.h>

namespace lu = linalg_utils;

namespace planner {

namespace cost_function {
  using Trajectory = std::deque<ForwardArcMotionPrimitives>;

  /*======================== PRIMITIVE COSTS ========================*/

  // StraightLine: penalizes curvature in the trajectory. This is implemented
  // in penalizing non-zero omega and velocity.
  inline float StraightLine(const ForwardArcMotionPrimitives& primitive) {

    float cost = 0.0;

    if (primitive.omega() != 0.0) cost += std::abs(primitive.omega());
    if (primitive.vel_z() != 0.0) cost += std::abs(primitive.vel_z());

    return cost;
  };

  // Direction: penalize trajectories that are too far away from the current heading
  inline float Direction(const ForwardArcMotionPrimitives& primitive, const FlatState &ref_state) {

    float deviation_cost = 0.0;
    std::vector<FlatState> path = primitive.samplePath(0.2);
    float normalized_ref_yaw = lu::NormalizeHeading((float)ref_state.yaw);
    for (auto &wpt : path)
    {
      float heading = std::atan2(wpt.pos.y() - ref_state.pos.y(), wpt.pos.x() - ref_state.pos.x());
      deviation_cost += std::abs(lu::HeadingDifference(heading, normalized_ref_yaw));
    }
    return deviation_cost;
  };

  // Duration: Penalize trajectories that have short segments
  inline float Duration(const ForwardArcMotionPrimitives& primitive) {


    float cost = 1.0 / primitive.duration();

    if (std::isnan(cost)) {
      std::cout << "[WARNING][Cost.h] Duration cost computes NaN" << std::endl;
      cost = 0;
    }

    return cost;
  };

  // Speed: Penalize slow trajectories
  inline float Speed(const ForwardArcMotionPrimitives& primitive)
  {


    float cost = 1 / (primitive.speed());

    if (std::isnan(cost)) {
      std::cout << "[WARNING][Cost.h] Speed cost computes NaN" << std::endl;
      cost = 0;
    } else if (std::isinf(cost)) {
      cost = 1000;
    }

    return cost;
  };

  // Point: Penalize distance to a point
  inline float Point(const ForwardArcMotionPrimitives& primitive, const Eigen::Vector3d& point, int N)
  {
    float cost = 0;
    float dt = primitive.duration() / (float)N;
    std::vector<FlatState> path = primitive.samplePath(dt);
    for (auto &wpt : path)
    {
      cost += (wpt.pos - point).norm();
    }
    // return cost / (float)path.size();
    return cost;
  };

  // Deviation penalize trajectories that are too far away from the centerline
 inline float Deviation(const ForwardArcMotionPrimitives& primitive) {

   float deviation_cost = 0.0;
   std::vector<FlatState> path = primitive.samplePath(0.1);
   for (auto &wpt : path)
   {
     deviation_cost += std::abs(wpt.pos.y());
     deviation_cost += std::abs(wpt.pos.z() - 1);
   }
   return deviation_cost;
 };


  /*======================== TRAJECTORY COSTS ========================*/

  // Smoothness: Penalize changes in curvature.
  inline float Smoothness(const Trajectory& trajectory) {
    float cost = 0.0;

    // Equally penalize 1 primitive trajectories
    if (trajectory.size() == 1) return 0.0;

    // Else, compute normal values.
    for (size_t i = 1; i < trajectory.size(); i++) {
      cost += std::abs(trajectory[i].omega() - trajectory[i-1].omega());
      cost += std::abs(trajectory[i].vel_x() - trajectory[i-1].vel_x());
      cost += std::abs(trajectory[i].vel_z() - trajectory[i-1].vel_z());
    }

    if (std::isnan(cost)) {
      std::cout << "[WARNING][Cost.h] Smoothness cost computes NaN" << std::endl;
      cost = 0;
    }
    if (std::isinf(cost)) {
      std::cout << "[WARNING][Cost.h] Smoothness cost computes infinity" << std::endl;
      for (auto & seg : trajectory) {
        std::cout << "(v = " << seg.vel_x() << ", omega = " << seg.omega() << ", z = " << seg.vel_z() << ")" << std::endl;
      }
      cost = 1000;
    }

    return cost;
  }

  // StraightLine: penalizes curvature in the trajectory. This is implemented
  // in penalizing non-zero omega and velocity.
  inline float StraightLine(const Trajectory& trajectory) {
    float cost = 0.0;

    for (auto &primitive : trajectory) {
      cost += StraightLine(primitive);
    }

    return cost;
  };

  // Direction: penalize trajectories that are too far away from the current heading. Normalized per segment.
  inline float Direction(const Trajectory& trajectory, const FlatState &ref_state) {
    float cost = 0.0;

    float trajectory_duration = 0;
    for (auto &primitive : trajectory) {
      cost += Direction(primitive, ref_state);
      trajectory_duration += primitive.duration();
    }

    // Normalize the cost by the duration of the trajectory
    cost = cost / trajectory_duration;

    return cost;
  };

  // Input: Compute cost of the trajectory with respect to a direction (given that the two inputs are specified in the same frame)
  inline float Input(const Trajectory& trajectory, const Eigen::Vector4d& input, float duration) {
    // Compute the dot product between the two vectors.

    // Create a primitive with the same duration. input is of the form [vx, omega, vz, vside]
    ForwardArcMotionPrimitives primitive(trajectory.front().getReferenceState(), input, duration, 0.3, true);
    float cost = 0;

    // Sum over pairwise differences
    // int seg_number = 0;
    // auto seg_duration = trajectory.front().duration(); // Duration up to seg_number, so far

    // int N = 20;
    // float dt = duration / (float)N;

    // auto difference = 0;

    // Eigen::Vector3d avg_vel;
    // Eigen::Vector4d avg_input;
    //
    // for (int i = 0; i < N; i++) {
    //   float t = i*dt;
    //
    //   // Jump to the next segent
    //   if (t > seg_duration) {
    //     seg_number++;
    //     seg_duration += trajectory[seg_number].duration();
    //   }
    //
    //   // Get pose from trajectory
    //   auto traj_pose = trajectory[seg_number].getWorldPoseAtTime(t);
    //   // // Get pose from primitive
    //   // auto primitive_pose = primitive.getWorldPoseAtTime(t);
    //
    //   // Compute pairwise difference
    //   avg_vel += traj_pose.vel;
    //   avg_input += trajectory[seg_number].input();
    // } // end loop over trajectory


    // cost = (cost.normalize() - primitive.getInitialPose().vel.normalize()).norm();

    // auto cproduct = gu::cross(avg_vel.normalize(), (primitive.getFinalLocalPose().pos - primitive.getInitialLocalPose().pos).normalize());

    auto traj_vector = (trajectory.back().getFinalWorldPose().pos - trajectory.front().getInitialWorldPose().pos).normalized();

    auto traj_primitive = (primitive.getFinalWorldPose().pos - primitive.getInitialWorldPose().pos).normalized();

    auto cproduct = 1.0 - traj_vector.dot(traj_primitive);
    cost = std::abs(cproduct);

    return cost;
  } // function Input

  inline float Input2(const Trajectory& trajectory, const Eigen::Vector3d& input_primitive) {
    // Compute the dot product between the two vectors.

    auto traj_vector = (trajectory.back().getFinalWorldPose().pos - trajectory.front().getInitialWorldPose().pos).normalized();

    auto cproduct = 1.0 - traj_vector.dot(input_primitive);
    float cost = std::abs(cproduct);

    return cost;
  } // function Input

  // Duration: Penalizes trajectories that have short segments
  inline float Duration(const Trajectory& trajectory) {
    float cost = 0.0;

    for (auto &primitive : trajectory) {
      cost += Duration(primitive);
    }

    // Normalize the cost by number of segments
    cost = cost / trajectory.size();

    if (std::isnan(cost)) {
      std::cout << "[WARNING][Cost.h] Trajectory duration cost computes NaN" << std::endl;
      cost = 0;
    }

    return cost;
  };

  // Speed: Penalize slow trajectories (1/avg_speed)
  inline float Speed(const Trajectory& trajectory) {

    float speed = 0;
    for (auto &primitive : trajectory) {
      speed += primitive.speed();
    }
    speed = speed / trajectory.size();

    float cost = 1/speed;
    return cost;
  };

  // AverageSpeed: Computes the average speed along a trajectory
  inline float AverageSpeed(const Trajectory& trajectory) {

    if (trajectory.size() == 1) return trajectory.front().vel_x();

    float v_avg = 0;

    // Else, compute normal values.
    for (size_t i = 0; i < trajectory.size(); i++) {
      v_avg = (v_avg * i + trajectory[i].speed()) / (i + 1);
    }

    if (std::isnan(v_avg)) {
      std::cout << "[WARNING][Cost.h] AverageSpeed cost computes NaN" << std::endl;
      v_avg = 0;
    }

    return v_avg;
  }

  // Compares the orthogonality between two trajectories.
  inline float Orthogonality(const std::deque<ForwardArcMotionPrimitives> traj1, const std::deque<ForwardArcMotionPrimitives>& traj2) {

      float c = 0;
      if (traj1.size() > 0 && traj2.size() > 0) {

        // Compute the derivatives to see how parallel they are.

        FlatState traj1_front = traj1.front().getInitialWorldPose();
        FlatState traj1_back = traj1.back().getFinalWorldPose();
        FlatState traj2_front = traj2.front().getInitialWorldPose();
        FlatState traj2_back = traj2.back().getFinalWorldPose();


        // A three point basis calculation
        // Eigen::Vector3d p_km1 = traj1_front.pos;
        // Eigen::Vector3d p_k = traj2_front.pos;
        // Eigen::Vector3d p_kp1 = traj2_back.pos;
        // c = ((p_k - p_km1).norm() + (p_kp1 - p_k).norm()) / ((p_kp1-p_km1).norm()) - 1;

        // Compute the cross product between the averaged two velocity vectors between the two trajectories.
        Eigen::Vector3d v1 = traj1_back.pos - traj1_front.pos;
        Eigen::Vector3d v2 = traj2_back.pos - traj2_front.pos;
        auto cproduct = v1.normalized().cross(v2.normalized());
        c = std::abs(cproduct.norm());
      }

      if (std::isnan(c)) {
        std::cout << "[WARNING][Cost.h] Orthogonality cost computes NaN" << std::endl;
        c = 0;
      }

      return c;
  };

  // Computes the distance between two trajectories (possibly real vs. optimal)
  inline float Distance(Eigen::MatrixXd& wpts1, Eigen::MatrixXd& wpts2)
  {

    double d = 0;

    if (wpts1.cols() != wpts2.cols())
      throw std::invalid_argument("[Distance] wpts need to be the same length!");

    for (int i = 1; i < wpts1.cols(); i++) {
      d += ( (wpts1.col(i)-wpts1.col(i-1)).norm() - (wpts2.col(i)-wpts2.col(i-1)).norm() )/ 2 * (wpts1.col(i)-wpts2.col(i)).norm();
    }

    if (std::isnan(d)) {
      std::cout << "[WARNING][Cost.h] Distance cost computes NaN" << std::endl;
      d = 0;
    }

    return d;
  };

  // Goal: penalize distance of the final point to the goal
  inline float Goal(const Trajectory& trajectory, const Eigen::Vector3d& goal) {
    return (trajectory.back().getFinalWorldPose().pos - goal).norm();
  }

  // Point: Penalize distance to a point
  inline float Point(const Trajectory& trajectory, const Eigen::Vector3d& point, const int N) {
    float cost = 0;
    int n = std::round((float)N/trajectory.size());
    for (auto& primitive : trajectory)
    {
      cost += Point(primitive, point, n);
    }
    // return cost / (float)trajectory.size();
    return cost;
  }

  // Length: penalize shorter distance trajectories
  inline float Length(const Trajectory& trajectory)
  {
    float dist = 0;
    for (auto & primitive : trajectory)
    {
      dist += primitive.speed() * primitive.duration();
    }
    float cost = 1.0 / dist;
    return cost;
  }

  // Deviation: deviation of the trajectory is defined as the distance between the final pose to the centerline, along the x axis!
  inline float Deviation(const Trajectory& trajectory) {
     float cost = 0.0;

     // float trajectory_duration = 0;
     // for (auto &primitive : trajectory) {
     //   cost += Deviation(primitive);
     //   trajectory_duration += primitive.duration();
     // }

     // Normalize the cost by the duration of the trajectory
     // cost = cost / trajectory_duration;
     cost = (trajectory.back().getFinalWorldPose().pos - Eigen::Vector3d(trajectory.back().getFinalWorldPose().pos(0), 0, trajectory.back().getFinalWorldPose().pos(2))).norm();
     return cost;
   };


} // namespace cost_function

} // namespace planner
