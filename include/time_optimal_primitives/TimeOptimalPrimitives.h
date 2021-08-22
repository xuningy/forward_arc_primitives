/*
time_optimal_primitives
Copyright (C) 2020 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <ros/ros.h>
#include <tuple>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <planning_representations/Trajectory.h>
#include <planning_representations/FlatState.h>
#include <trajectory_calculators/TimeOptimalPolynomialCalculator.h>
#include <trajectory_calculators/PolynomialCalculator.h>

namespace planner {

/* Implementation of time optimal motion primitives from:
A Computationally Efficient Motion Primitive for Quadrocopter Trajectory Generation
https://ieeexplore.ieee.org/document/7299672
from Mark W. Mueller, Markus Hehn, Raffaello D'Andrea.
*/
class TimeOptimalPrimitives : public Trajectory
{
  public:

    typedef std::shared_ptr<TimeOptimalPrimitives> Ptr;
    typedef std::shared_ptr<const TimeOptimalPrimitives> ConstPtr;

    using Trajectory::samplePath;
    using Trajectory::samplePathInLocalFrame;

    TimeOptimalPrimitives();
    TimeOptimalPrimitives(const FlatState& curr_ref,
      const Eigen::Vector4d& input,
      double tf);
    ~TimeOptimalPrimitives();

    void setStartTime(const double t0) override { t0_ = t0; };
    void setEndTime(const double tf) override { tf_ = tf; };

    double omega() const { return omega_; };
    double vel_x() const { return vx_; };
    double vel_y() const { return vy_; };
    double vel_z() const { return vz_; };
    double vel_body() const { return xy_speed_; };
    double speed() const { return speed_; };
    double duration() const override { return tf_ - t0_; };
    double tstart() const override{ return t0_; };
    double tfinal() const override { return tf_; };
    FlatState getReferenceState() const { return curr_ref_w_; };
    Eigen::Vector4d input() const { return input_; };

    std::vector<FlatState> samplePath(double dt, double tstart, double tfinal) const override;
    std::vector<FlatState> samplePath(double dt, double tfinal) const
      { return samplePath(dt, tstart(), tfinal); };

    std::vector<FlatState> samplePathInLocalFrame(double dt, double tstart, double tfinal) const override;
    std::vector<FlatState> samplePathInLocalFrame(double dt, double tfinal) const { return samplePathInLocalFrame(dt, tstart(), tfinal); };

    FlatState getWorldPoseAtTime(double t) const override { return getPoseAtTime(t, x_abc_w_, y_abc_w_, z_abc_w_, curr_ref_w_, yaw_coeff_w_); };
    FlatState getLocalPoseAtTime(double t) const override { return getPoseAtTime(t, x_abc_b_, y_abc_b_, z_abc_b_, curr_ref_b_, yaw_coeff_b_); };
    FlatState getPoseAtTime(double t, const Eigen::Vector3d& x_abc, const Eigen::Vector3d& y_abc, const Eigen::Vector3d& z_abc, const FlatState& curr_ref, const Vec8_t& coeff_yaw) const;

    void print(bool verbose = false) const;


    bool is_initialized_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:

    Eigen::Vector4d input_;

    double speed_, xy_speed_;
    double vx_, vz_, vy_, omega_;
    double t0_ = 0, tf_;

    // The initial transform represents the transform produced by the current
    // trajectory reference at the time that this motion-primitive object will
    // be instantiated.
    // geometry_utils::Transform3d initial_tsf_;
    FlatState curr_ref_w_;
    FlatState curr_ref_b_;
    double yaw_offset_;

    Vec8_t yaw_coeff_b_;
    Vec8_t yaw_coeff_w_;

    // Eigen::Vector3d perAxisTraj(Eigen::Vector3d& abc, double t, double p0, double v0, double a0);
    //
    // Eigen::Vector3d alphaBetaGamma(double T, double p0, double v0, double a0, double pf, double vf, double af, Constraint c);

    Eigen::Vector3d x_abc_w_, y_abc_w_, z_abc_w_;
    Eigen::Vector3d x_abc_b_, y_abc_b_, z_abc_b_;

};

} // namespace planner
