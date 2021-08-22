/*
forward_arc_primitives: snap-continuous motion primitives with unicycle dynamics for multirotor air vehicles.
Copyright (C) 2018 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <forward_arc_primitives/ForwardArcMotionPrimitives.h>

namespace planner {

ForwardArcMotionPrimitives::ForwardArcMotionPrimitives() : is_initialized_(0)
{
  input_ = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
  tf_ = 0;
}

ForwardArcMotionPrimitives::~ForwardArcMotionPrimitives() {}

ForwardArcMotionPrimitives::ForwardArcMotionPrimitives(
  const FlatState& curr_ref_w, const Eigen::Vector4d& input, double tf) :
  ForwardArcMotionPrimitives(curr_ref_w, input, tf, tf, false) { }


static PolynomialCalculator traj_calc;

ForwardArcMotionPrimitives::ForwardArcMotionPrimitives(
  const FlatState& curr_ref_w, const Eigen::Vector4d& input,
  double tf, double yaw_tf, bool aligned_heading)
{
  // Add mode for fixed_acceleration
  mode_ = "!fixed_acceleration"; // Undeveloped feature

  input_ = input;
  aligned_heading_ = aligned_heading;

  // side velocity deadband with 0.03
  input_(3) = std::abs(input_(3)) < 0.03 ? 0 : input_(3);

  // Set inputs, speed, etc.
  vx_ = input_(0);
  vy_ = input_(3);
  vz_ = input_(2);
  omega_ = input_(1);
  xy_speed_= std::abs(vy_) < 1e-2? vx_ : std::sqrt(vx_*vx_ + vy_*vy_);
  speed_= std::sqrt(vx_*vx_ + vy_*vy_ + vz_*vz_);

  // Set timing parameters.
  tf_ = tf;
  yaw_tf_ = yaw_tf;

  // Throw a warning of the trajectory length is less than 50ms.
  if (tf < 0.05)
  {
    ROS_WARN("[ForwardArcMotionPrimitives] Duration of specified primitive (%.1fms) is less than 50ms!", tf*100);
  }


  // Set transforms
  curr_ref_w_ = curr_ref_w;
  // initial_tsf_.translation = curr_ref_w.pos;
  // initial_tsf_.rotation = curr_ref_w.rot;
  translation_offset_ = curr_ref_w.pos;  // translation
  yaw_offset_ = curr_ref_w.yaw;  // yaw
  Eigen::Affine3d initial_tsf;
  initial_tsf = Eigen::AngleAxisd(curr_ref_w.yaw, Eigen::Vector3d::UnitZ());

  // compute an 8th order polynomial up to velocity


  // in the body frame
  Eigen::Vector3d curr_vel_b = initial_tsf.inverse() * curr_ref_w_.vel;
  Eigen::Vector3d curr_acc_b = initial_tsf.inverse() * curr_ref_w_.acc;
  Eigen::Vector3d curr_jerk_b = initial_tsf.inverse() * curr_ref_w_.jerk;
  Eigen::Vector3d curr_snap_b = initial_tsf.inverse() * curr_ref_w_.snap;

  end_point_t start[4] = {
    { curr_vel_b(0), curr_acc_b(0), curr_jerk_b(0), curr_snap_b(0) },
    { curr_vel_b(1), curr_acc_b(1), curr_jerk_b(1), curr_snap_b(1) },
    { curr_vel_b(2), curr_acc_b(2), curr_jerk_b(2), curr_snap_b(2) },
    { curr_ref_w_.dyaw, 0, 0, 0 } };


  if (mode_ == "fixed_acceleration" && !std::isnan(curr_vel_b.norm())) {
    t_ramp_ = std::min(std::abs(xy_speed_- curr_vel_b.norm()) / 3, tf);
    t_coast_ = tf - t_ramp_;
    std::cout << "xy_speed_" << xy_speed_<< " curr_vel_b: (" << curr_vel_b;
    std::cout << ") with norm: " << curr_vel_b.norm() << " t_ramp_: " << t_ramp_ << " t_coast_: " << t_coast_ << std::endl;

    if (t_ramp_ < 0.1) {
      t_ramp_ = 0;
      t_coast_ = tf;
    }
  } else {
    t_ramp_ = tf;
    t_coast_ = 0;
  }

  double v = xy_speed_;
  double omega = input_(1);
  double th = omega * t_ramp_;
  double zv = input_(2);

  // Rotate the primitive wrt to the joystick angle (e.g. side velocity)
  double alpha = (std::abs(std::atan2(vy_, vx_) - 3.1415) < 0.001) ? 0 : std::atan2(vy_, vx_);
  Eigen::Rotation2Dd Rs(alpha);

  Eigen::Vector2d vel_f;
  vel_f << v * cos(th),
           v * sin(th);
  vel_f = Rs * vel_f;

  Eigen::Vector2d acc_f;
  acc_f << -v * sin(th) * omega,
            v * cos(th) * omega;
  acc_f = Rs * acc_f;

  Eigen::Vector2d jerk_f;
  jerk_f << -v * cos(th) * omega * omega,
            -v * sin(th) * omega * omega;
  jerk_f = Rs * jerk_f;

  end_point_t end[4] = { {vel_f(0), acc_f(0), jerk_f(0), 0 },
                      { vel_f(1), acc_f(1), jerk_f(1), 0 },
                      { zv, 0, 0, 0 },
                      { omega, 0, 0, 0 } };

  //coeff_vel_b_ = traj_calc.get_poly4(start, end, t_ramp_);
  traj_calc.get_poly4(start, end, t_ramp_, coeff_vel_b_);

  if (aligned_heading_) {
    end_point_t yaw_start = {curr_ref_w_.yaw, curr_ref_w_.dyaw, 0, 0};
    end_point_t yaw_end = {omega * yaw_tf_ + yaw_offset_, omega, 0, 0};
    Vec8_t coeff_vel_yaw = traj_calc.get_poly(yaw_start, yaw_end, yaw_tf_);
    coeff_yaw_w_.block(0, 0, 8, 1) = coeff_vel_yaw.array();
    coeff_yaw_w_(8) = 0;
  }

  // compute transform to the world frame
  Eigen::Matrix<double, 3, 3> R = Eigen::MatrixXd::Identity(3, 3)*Eigen::AngleAxisd(yaw_offset_, Eigen::Vector3d::UnitZ());

  // adjust coefficients so that we have a 9th order polynomial in position
  // by multiplying each body coefficient by 1, 1/2, 1/3, 1/4, ... 1/8
  Eigen::Matrix<double, 8, 1> coeff_adjust;
  coeff_adjust << 1.0, 0.5, 0.33333333, 0.25, 0.2, 0.16666666666, 0.14285714285, 0.125;

  // Transform them to the world frame
  Eigen::Matrix<double, 9, 3> pos_b;
  pos_b.row(0) = Eigen::Matrix<double, 1, 3>::Zero();
  pos_b.block(1, 0, 8, 1) = coeff_vel_b_[0].array()*coeff_adjust.array();
  pos_b.block(1, 1, 8, 1) = coeff_vel_b_[1].array()*coeff_adjust.array();
  pos_b.block(1, 2, 8, 1) = coeff_vel_b_[2].array()*coeff_adjust.array();
  Eigen::Matrix<double, 9, 3> pos_w = (R*pos_b.transpose()).transpose();
  pos_w.row(0) = translation_offset_;

  Eigen::Matrix<double, 9, 1> yaw_w;
  yaw_w.block(1, 0, 8, 1) = coeff_vel_b_[3].array()*coeff_adjust.array();

  coeff_b_.push_back(pos_b.col(0).transpose());
  coeff_b_.push_back(pos_b.col(1).transpose());
  coeff_b_.push_back(pos_b.col(2).transpose());
  coeff_b_.push_back(yaw_w);

  yaw_w(0) = yaw_offset_;

  coeff_w_.push_back(pos_w.col(0).transpose());
  coeff_w_.push_back(pos_w.col(1).transpose());
  coeff_w_.push_back(pos_w.col(2).transpose());
  coeff_w_.push_back(yaw_w);

  is_initialized_ = true;

  if (mode_ == "fixed_acceleration" && t_coast_ > 0.01)
  {
    double th2 = omega * tf;

    FlatState end_pose_seg1;
    FlatState end_pose_seg1_w;

    double end_pose_yaw, end_pose_dyaw;
    Eigen::Vector3d end_pose_pos;

    if (t_ramp_ > 0.01) {
      // Get final pose from prev. segment.
      end_pose_seg1 = getPoseAtTime(t_ramp_, coeff_b_);
      end_pose_seg1_w = getPoseAtTime(t_ramp_, coeff_w_);

      end_pose_yaw = end_pose_seg1_w.yaw;
      end_pose_dyaw = end_pose_seg1_w.dyaw;

      end_pose_pos = end_pose_seg1_w.pos;

    } else {
      end_pose_pos = translation_offset_;
      end_pose_seg1.vel = curr_vel_b;
      end_pose_seg1.acc = curr_acc_b;
      end_pose_seg1.jerk = curr_jerk_b;
      end_pose_seg1.snap = curr_snap_b;
      end_pose_dyaw = curr_ref_w_.dyaw;
      th = 0;
    }

    // std::cout << "acc (" << end_pose_seg1.acc <<") ";
    end_point_t start2[4] = {
      { end_pose_seg1.vel(0), end_pose_seg1.acc(0), end_pose_seg1.jerk(0), 0},
      { end_pose_seg1.vel(1), end_pose_seg1.acc(1), end_pose_seg1.jerk(1), 0},
      { end_pose_seg1.vel(2), end_pose_seg1.acc(2), end_pose_seg1.jerk(2), 0},
      { end_pose_dyaw, 0, 0, 0 } };

    Eigen::Vector2d vel_f2;
    vel_f2 << v * cos(th2),
              v * sin(th2);
    vel_f2 = Rs * vel_f2;

    Eigen::Vector2d acc_f2;
    acc_f2 << -v * sin(th2) * omega,
               v * cos(th2) * omega;
    acc_f2 = Rs * acc_f2;

    Eigen::Vector2d jerk_f2;
    jerk_f2 << -v * cos(th2) * omega * omega,
               -v * sin(th2) * omega * omega;
    jerk_f2 = Rs * jerk_f2;

    end_point_t end2[4] = { {vel_f2(0), acc_f2(0), jerk_f2(0), 0 },
                        { vel_f2(1), acc_f2(1), jerk_f2(1), 0 },
                        { zv, 0, 0, 0 },
                        { omega, 0, 0, 0 } };

    coeff_vel_b_seg2_ = traj_calc.get_poly4(start2, end2, t_coast_);

    // compute transform from the navigation frame to the world frame
    Eigen::Matrix<double, 3, 3> R = Eigen::MatrixXd::Identity(3, 3)*Eigen::AngleAxisd(yaw_offset_, Eigen::Vector3d::UnitZ());

    // Transform coefficients to the world frame
    Eigen::Matrix<double, 9, 3> pos_b_seg2;
    pos_b_seg2.row(0) = Eigen::Matrix<double, 1, 3>::Zero();
    pos_b_seg2.block(1, 0, 8, 1) = coeff_vel_b_seg2_[0].array()*coeff_adjust.array();
    pos_b_seg2.block(1, 1, 8, 1) = coeff_vel_b_seg2_[1].array()*coeff_adjust.array();
    pos_b_seg2.block(1, 2, 8, 1) = coeff_vel_b_seg2_[2].array()*coeff_adjust.array();
    Eigen::Matrix<double, 9, 3> pos_w_seg2 = (R*pos_b_seg2.transpose()).transpose();

    pos_w_seg2.row(0) = end_pose_pos;

    Eigen::Matrix<double, 9, 1> yaw_w_seg2;
    yaw_w_seg2.block(1, 0, 8, 1) = coeff_vel_b_seg2_[3].array()*coeff_adjust.array();

    coeff_b_seg2_.push_back(pos_b_seg2.col(0).transpose());
    coeff_b_seg2_.push_back(pos_b_seg2.col(1).transpose());
    coeff_b_seg2_.push_back(pos_b_seg2.col(2).transpose());
    coeff_b_seg2_.push_back(yaw_w_seg2);

    yaw_w_seg2(0) = end_pose_yaw;

    coeff_w_seg2_.push_back(pos_w_seg2.col(0).transpose());
    coeff_w_seg2_.push_back(pos_w_seg2.col(1).transpose());
    coeff_w_seg2_.push_back(pos_w_seg2.col(2).transpose());
    coeff_w_seg2_.push_back(yaw_w_seg2);

  }

}

std::vector<FlatState> ForwardArcMotionPrimitives::samplePathInLocalFrame(double dt, double tstart, double tfinal) const
{
  std::vector<FlatState> traj_path;
  // tfinal = std::min(tf_, tfinal);
  tstart = std::max(t0_, tstart);

  if (mode_ == "fixed_acceleration" && t_coast_ > 0.01)
  {
    for (double t = tstart; t <= tfinal; t+=dt)
    {
      if (t < t_ramp_) {
        traj_path.push_back(getPoseAtTime(t, coeff_b_));
      } else if (t >= t_ramp_ && t <= tf_) {
        FlatState pose = getPoseAtTime(t - t_ramp_, coeff_b_seg2_);
        pose.t = t;
        traj_path.push_back(pose);
      }
    } // end for
  } else {
    for (double t = tstart; t <= tfinal; t+=dt)
      traj_path.push_back(getLocalPoseAtTime(t));
  } // end if

  return traj_path;
}

std::vector<FlatState> ForwardArcMotionPrimitives::samplePath(double dt, double tstart, double tfinal) const
{
  std::vector<FlatState> traj_path;
  // tfinal = std::min(tf_, tfinal);
  tstart = std::max(t0_, tstart);

  if (mode_ == "fixed_acceleration" && t_coast_ > 0.01)
  {
    for (double t = tstart; t <= tfinal; t+=dt)
    {
      if (t < t_ramp_) {
        traj_path.push_back(getPoseAtTime(t, coeff_w_));
      } else if (t >= t_ramp_ && t <= tf_) {
        FlatState pose = getPoseAtTime(t - t_ramp_, coeff_w_seg2_);
        pose.t = t;
        traj_path.push_back(pose);
      }
    } // end for
  } else {
    for (double t = tstart; t <= tfinal; t+=dt)
      traj_path.push_back(getWorldPoseAtTime(t));
  } // end if

  return traj_path;
}

std::vector<FlatState> ForwardArcMotionPrimitives::samplePathEquidistant(double ds) const
{
  std::vector<FlatState> traj_path;
  double dt = 0.001;
  FlatState prev_state = getWorldPoseAtTime(t0_);
  traj_path.push_back(prev_state);
  for (double t = t0_; t <= tf_; t+=dt)
  {
    FlatState curr_state = getWorldPoseAtTime(t);
    if ((curr_state.pos - prev_state.pos).norm() > ds)
    {
      traj_path.push_back(curr_state);
      prev_state = curr_state;
    }
  }
  // Theres no special treatment for the last value at tf (so it may not be added.)

  return traj_path;
}

FlatState ForwardArcMotionPrimitives::getPoseAtTime(double t, const Poly9& coeff) const
{
  double tp2 = t*t;
  double tp3 = tp2*t;
  double tp4 = tp3*t;
  double tp5 = tp4*t;
  double tp6 = tp5*t;
  double tp7 = tp6*t;
  double tp8 = tp7*t;

  Eigen::Matrix<double, 5, 9> time_matrix;
  time_matrix <<  1, t, tp2, tp3, tp4, tp5, tp6, tp7, tp8,
            0, 1, 2*t, 3*tp2, 4*tp3, 5*tp4, 6*tp5, 7*tp6, 8*tp7,
            0, 0, 2, 6*t, 12*tp2, 20*tp3, 30*tp4, 42*tp5, 56*tp6,
            0, 0, 0, 6, 24*t, 60*tp2, 120*tp3, 210*tp4, 336*tp5,
            0, 0, 0, 0, 24, 120*t, 360*tp2, 840*tp3, 1680*tp4;

  Eigen::Matrix<double, 5, 1> x_pose, y_pose, z_pose, yaw_pose;
  x_pose = time_matrix * coeff[0];
  y_pose = time_matrix * coeff[1];
  z_pose = time_matrix * coeff[2];
  yaw_pose = time_matrix * coeff[3];

  FlatState pose;
  pose.t = t;

  pose.pos = Eigen::Vector3d(x_pose(0), y_pose(0), z_pose(0));
  pose.vel = Eigen::Vector3d(x_pose(1), y_pose(1), z_pose(1));
  pose.acc = Eigen::Vector3d(x_pose(2), y_pose(2), z_pose(2));
  pose.jerk = Eigen::Vector3d(x_pose(3), y_pose(3), z_pose(3));
  pose.snap = Eigen::Vector3d(x_pose(4), y_pose(4), z_pose(4));

  bool is_pure_yaw = std::abs(x_pose(1)) < 0.01 && std::abs(y_pose(1)) < 0.01;

  // if (is_pure_yaw || !aligned_heading_)
  // {
  //   pose.yaw(yaw_pose(0));
  //   pose.dyaw = yaw_pose(1);
  //   pose.d2yaw = 0;
  // } else
  // {
    // hack: a double polynomial for this trajectory
    if (t <= yaw_tf_) {
      yaw_pose = time_matrix * coeff_yaw_w_;
      pose.yaw = yaw_pose(0);
      pose.dyaw = yaw_pose(1);
      pose.ddyaw = 0;
    } else {
      pose.yaw = omega_ * t + yaw_offset_;
      pose.dyaw = omega_;
      pose.ddyaw = 0;
    }
  // }


  return pose;
}
void ForwardArcMotionPrimitives::print(bool verbose) const
{
  std::cout << std::setprecision(2) << "Motion Primitive" <<
  " linear=" << input_(0) << " omega=" << input_(1) <<" z=" << input_(2) << " side=" << input_(3) << " speed=" << xy_speed_ << " duration=" << duration()  << " yaw_rampup_duration=" << yaw_tf_ <<  std::endl;

  if (!verbose) return;

  std::vector<FlatState> poses = samplePath(0.1);
  std::cout << "===========================WORLD===============================" << std::endl;
  std::cout << "\t pos \t   |\t\t vel \t   |   \t acc \t   |  \t jerk \t   |  \t snap \t   | yaw  | yawdot " << std::endl;
  for (auto & pose : poses)
  {
  std::cout << std::setprecision(3) << std::fixed
            << pose.pos(0) << " " << pose.pos(1) << " " << pose.pos(2) << "|"
            << pose.vel(0) << " " << pose.vel(1) << " " << pose.vel(2) << "|"
            << pose.acc(0) << " " << pose.acc(1) << " " << pose.acc(2) << "|"
            << pose.jerk(0) << " " << pose.jerk(1) << " " << pose.jerk(2) << "|"
            << pose.snap(0) << " " << pose.snap(1) << " " << pose.snap(2) << "|"
            << pose.yaw << "|"
            << pose.dyaw << std::endl;
  }
  poses = samplePathInLocalFrame(0.1);
  std::cout << "===========================BODY===============================" << std::endl;
  std::cout << "\t pos \t   |\t\t vel \t   |   \t acc \t   |  \t jerk \t   |  \t snap \t   | yaw  | yawdot " << std::endl;
  for (auto & pose : poses)
  {
  std::cout << std::setprecision(3) << std::fixed
            << pose.pos(0) << " " << pose.pos(1) << " " << pose.pos(2) << "|"
            << pose.vel(0) << " " << pose.vel(1) << " " << pose.vel(2) << "|"
            << pose.acc(0) << " " << pose.acc(1) << " " << pose.acc(2) << "|"
            << pose.jerk(0) << " " << pose.jerk(1) << " " << pose.jerk(2) << "|"
            << pose.snap(0) << " " << pose.snap(1) << " " << pose.snap(2) << "|"
            << pose.yaw << "|"
            << pose.dyaw << std::endl;
  }
  std::cout << "===============================================================" << std::endl;
}

} // namespace planner
