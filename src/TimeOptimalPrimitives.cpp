/*
time_optimal_primitives
Copyright (C) 2020 Xuning Yang

Redistribution and use in source and binary forms, with or without modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the following disclaimer in the documentation and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote products derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/
#include <time_optimal_primitives/TimeOptimalPrimitives.h>

namespace planner {

/* Implementation of time optimal motion primitives from:
A Computationally Efficient Motion Primitive for Quadrocopter Trajectory Generation
https://ieeexplore.ieee.org/document/7299672
from Mark W. Mueller, Markus Hehn, Raffaello D'Andrea.
*/

static TimeOptimalPolynomialCalculator time_optimal_poly_calc;
static PolynomialCalculator traj_calc;

TimeOptimalPrimitives::TimeOptimalPrimitives() : is_initialized_(0)
{
  input_ = Eigen::Vector4d(0.0, 0.0, 0.0, 0.0);
  tf_ = 0;
}

TimeOptimalPrimitives::~TimeOptimalPrimitives() {}

TimeOptimalPrimitives::TimeOptimalPrimitives(
  const FlatState& curr_ref_w, const Eigen::Vector4d& input,
  double tf)
{
  input_ = input;

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

  // Throw a warning of the trajectory length is less than 50ms.
  if (tf < 0.05)
  {
    ROS_WARN("[TimeOptimalPrimitives] Duration of specified primitive (%.1fms) is less than 50ms!", tf*100);
  }


  // Set transforms
  curr_ref_w_ = curr_ref_w;
  // initial_tsf_.translation = curr_ref_w.pos;
  // initial_tsf_.rotation = curr_ref_w.rot;
  // translation_offset_ = curr_ref_w.pos;  // translation
  yaw_offset_ = curr_ref_w.yaw;  // yaw
  Eigen::Affine3d initial_tsf;
  initial_tsf = Eigen::AngleAxisd(curr_ref_w.yaw, Eigen::Vector3d::UnitZ());

  // Compute the end point velocities
  double v = xy_speed_;
  double omega = input_(1);
  double th = omega * tf_;
  double zv = input_(2);

  // Rotate the primitive wrt to the joystick angle (e.g. side velocity)
  double alpha = (std::abs(std::atan2(vy_, vx_) - 3.1415) < 0.001) ? 0 : std::atan2(vy_, vx_);
  Eigen::Rotation2Dd R(yaw_offset_);
  Eigen::Rotation2Dd Rs(alpha);

  Eigen::Vector2d vel_f_b;
  vel_f_b << v * cos(th),
           v * sin(th);
  vel_f_b = Rs * vel_f_b;
  Eigen::Vector2d vel_f_w = R * Rs * vel_f_b;

  Eigen::Vector2d acc_f_b;
  acc_f_b << -v * sin(th) * omega,
            v * cos(th) * omega;
  acc_f_b = Rs * acc_f_b;
  Eigen::Vector2d acc_f_w = R * Rs * acc_f_b;

  /* world frame */

  // Compute alpha, beta, gamma x, y, z trajectories in the world frame
  // (note p0 is NaN because final position is unconstrained)
  x_abc_w_ = time_optimal_poly_calc.alphaBetaGamma(tf_, curr_ref_w_.pos(0), curr_ref_w_.vel(0), curr_ref_w_.acc(0), NAN, vel_f_w(0), acc_f_w(0), TimeOptimalPolynomialCalculator::VEL_ACC);
  y_abc_w_ = time_optimal_poly_calc.alphaBetaGamma(tf_, curr_ref_w_.pos(1), curr_ref_w_.vel(1), curr_ref_w_.acc(1), NAN, vel_f_w(1), acc_f_w(1), TimeOptimalPolynomialCalculator::VEL_ACC);
  z_abc_w_ = time_optimal_poly_calc.alphaBetaGamma(tf_, curr_ref_w_.pos(2), curr_ref_w_.vel(2), curr_ref_w_.acc(2), NAN, zv, 0.0, TimeOptimalPolynomialCalculator::VEL_ACC);

  // Compute yaw trajectory
  end_point_t yaw_w_0 = { curr_ref_w_.yaw, curr_ref_w_.dyaw, 0, 0 };
  end_point_t yaw_w_f = { curr_ref_w_.yaw+omega*tf_, omega, 0, 0};

  yaw_coeff_w_ = traj_calc.get_poly(yaw_w_0, yaw_w_f, tf_);

  /* body frame*/

  // Transform velocities into body frame
  curr_ref_b_.pos = Eigen::Vector3d(0, 0, 0);
  curr_ref_b_.vel = initial_tsf.inverse() * curr_ref_w_.vel;
  curr_ref_b_.acc = initial_tsf.inverse() * curr_ref_w_.acc;

  // Compute alpha, beta, gamma x, y, z trajectories in the body frame
  // (note p0 is NAN because final position is unconstrained)

  x_abc_b_ = time_optimal_poly_calc.alphaBetaGamma(tf_, 0.0, curr_ref_b_.vel(0), curr_ref_b_.acc(0), NAN, vel_f_b(0), acc_f_b(0), TimeOptimalPolynomialCalculator::VEL_ACC);
  y_abc_b_ = time_optimal_poly_calc.alphaBetaGamma(tf_, 0.0, curr_ref_b_.vel(1), curr_ref_b_.acc(1), NAN, vel_f_b(1), acc_f_b(1), TimeOptimalPolynomialCalculator::VEL_ACC);
  z_abc_b_ = time_optimal_poly_calc.alphaBetaGamma(tf_, 0.0, curr_ref_b_.vel(2), curr_ref_b_.acc(2), NAN, zv, 0.0, TimeOptimalPolynomialCalculator::VEL_ACC);

  // Compute yaw trajectory
  end_point_t yaw_b_0 = { 0, curr_ref_w_.dyaw, 0, 0 };
  end_point_t yaw_b_f = { omega*tf_, omega, 0, 0};

  yaw_coeff_b_ = traj_calc.get_poly(yaw_b_0, yaw_b_f, tf_);

  is_initialized_ = true;

}

std::vector<FlatState> TimeOptimalPrimitives::samplePathInLocalFrame(double dt, double tstart, double tfinal) const
{
  std::vector<FlatState> traj_path;
  // tfinal = std::min(tf_, tfinal);
  tstart = std::max(t0_, tstart);

  for (double t = tstart; t <= tfinal; t+=dt)
    traj_path.push_back(getLocalPoseAtTime(t));

  return traj_path;
}

std::vector<FlatState> TimeOptimalPrimitives::samplePath(double dt, double tstart, double tfinal) const
{
  std::vector<FlatState> traj_path;
  // tfinal = std::min(tf_, tfinal);
  tstart = std::max(t0_, tstart);

  for (double t = tstart; t <= tfinal; t+=dt)
    traj_path.push_back(getWorldPoseAtTime(t));

  return traj_path;
}

FlatState TimeOptimalPrimitives::getPoseAtTime(double t, const Eigen::Vector3d& x_abc, const Eigen::Vector3d& y_abc, const Eigen::Vector3d& z_abc, const FlatState& curr_ref, const Vec8_t& coeff_yaw) const
{
  double tp2 = t*t;
  double tp3 = tp2*t;
  double tp4 = tp3*t;
  double tp5 = tp4*t;
  double tp6 = tp5*t;
  double tp7 = tp6*t;

  Eigen::Matrix<double, 3, 8> time_matrix;
  time_matrix <<  1, t, tp2, tp3, tp4, tp5, tp6, tp7,
            0, 1, 2*t, 3*tp2, 4*tp3, 5*tp4, 6*tp5, 7*tp6,
            0, 0, 2, 6*t, 12*tp2, 20*tp3, 30*tp4, 42*tp5;

  Eigen::Matrix<double, 3, 1> yaw_pose;
  yaw_pose = time_matrix * coeff_yaw;

  Eigen::Vector3d x = time_optimal_poly_calc.perAxisTraj(x_abc, t, curr_ref.pos(0), curr_ref.vel(0), curr_ref.acc(0));
  Eigen::Vector3d y = time_optimal_poly_calc.perAxisTraj(y_abc, t, curr_ref.pos(1), curr_ref.vel(1), curr_ref.acc(1));
  Eigen::Vector3d z = time_optimal_poly_calc.perAxisTraj(z_abc, t, curr_ref.pos(2), curr_ref.vel(2), curr_ref.acc(2));

  FlatState pose;
  pose.t = t;
  pose.pos = Eigen::Vector3d(x(0), y(0), z(0));
  pose.vel = Eigen::Vector3d(x(1), y(1), z(1));
  pose.acc = Eigen::Vector3d(x(2), y(2), z(2));
  pose.jerk = Eigen::Vector3d(0, 0, 0);
  pose.snap = Eigen::Vector3d(0, 0, 0);

  pose.yaw = yaw_pose(0);
  pose.dyaw = yaw_pose(1);

  return pose;
}

void TimeOptimalPrimitives::print(bool verbose) const
{
  std::cout << std::setprecision(2) << "Motion Primitive" <<
  " linear=" << input_(0) << " omega=" << input_(1) <<" z=" << input_(2) << " side=" << input_(3) << " speed=" << xy_speed_ << " duration=" << duration()  <<  std::endl;

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
