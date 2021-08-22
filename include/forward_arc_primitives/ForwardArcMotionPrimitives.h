/*
forward_arc_primitives: snap-continuous motion primitives with unicycle dynamics for multirotor air vehicles.
Copyright (C) 2018 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef FORWARD_ARC_MOTION_PRIMITIVES_H
#define FORWARD_ARC_MOTION_PRIMITIVES_H

#include <ros/ros.h>
#include <tuple>
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <trajectory_calculators/PolynomialCalculator.h>
#include <planning_representations/Trajectory.h>
#include <planning_representations/FlatState.h>

namespace planner {

class ForwardArcMotionPrimitives : public Trajectory
{
  public:

    typedef std::vector<Vec9_t, Eigen::aligned_allocator<Vec9_t>> Poly9;
    typedef std::shared_ptr<ForwardArcMotionPrimitives> Ptr;
    typedef std::shared_ptr<const ForwardArcMotionPrimitives> ConstPtr;

    using Trajectory::samplePath;
    using Trajectory::samplePathInLocalFrame;

    ForwardArcMotionPrimitives();
    ForwardArcMotionPrimitives(const FlatState& curr_ref,
      const Eigen::Vector4d& input,
      double tf);
    ForwardArcMotionPrimitives(const FlatState& curr_ref,
      const Eigen::Vector4d& input,
      double tf, double yaw_tf, bool aligned_heading);
    ~ForwardArcMotionPrimitives();

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
    double yaw_rampup_duration() const { return yaw_tf_; };
    FlatState getReferenceState() const { return curr_ref_w_; };
    Eigen::Vector4d input() const { return input_; };

    std::vector<FlatState> samplePath(double dt, double tstart, double tfinal) const override;
    std::vector<FlatState> samplePath(double dt, double tfinal) const
      { return samplePath(dt, tstart(), tfinal); };
    std::vector<FlatState> samplePathEquidistant(double ds) const;

    std::vector<FlatState> samplePathInLocalFrame(double dt, double tstart, double tfinal) const override;
    std::vector<FlatState> samplePathInLocalFrame(double dt, double tfinal) const { return samplePathInLocalFrame(dt, tstart(), tfinal); };

    FlatState getWorldPoseAtTime(double t) const override { return getPoseAtTime(t, coeff_w_); };
    FlatState getLocalPoseAtTime(double t) const override { return getPoseAtTime(t, coeff_b_); };
    FlatState getPoseAtTime(double t, const Poly9& coeff) const;

    void print(bool verbose = false) const;


    bool is_initialized_;

    std::vector<Vec8_t, Eigen::aligned_allocator<Vec8_t>> coeff_vel_b_;
    Poly9 coeff_b_, coeff_w_;
    Vec9_t coeff_yaw_w_;

    std::vector<Vec8_t, Eigen::aligned_allocator<Vec8_t>> coeff_vel_b_seg2_;
    Poly9 coeff_b_seg2_, coeff_w_seg2_;
    Vec9_t coeff_yaw_w_seg2_;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:

    std::string mode_;
    bool aligned_heading_;

    Eigen::Vector4d input_;

    double speed_, xy_speed_;
    double vx_, vz_, vy_, omega_;
    double t0_ = 0, tf_, yaw_tf_;

    double t_ramp_, t_coast_;

    // The initial transform represents the transform produced by the current
    // trajectory reference at the time that this motion-primitive object will
    // be instantiated.
    // geometry_utils::Transform3d initial_tsf_;
    FlatState curr_ref_w_;
    Eigen::Vector3d translation_offset_;
    double yaw_offset_;

    Vec9_t x_coeff_b_, y_coeff_b_, z_coeff_b_, yaw_coeff_b_;
    Vec9_t x_coeff_w_, y_coeff_w_, z_coeff_w_, yaw_coeff_w_;





};

} // namespace planner
#endif
