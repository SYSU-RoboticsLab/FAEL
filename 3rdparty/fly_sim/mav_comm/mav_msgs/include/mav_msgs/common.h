/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Helen Oleynikova, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Common conversion functions between geometry messages, Eigen types, and yaw.

#ifndef MAV_MSGS_COMMON_H
#define MAV_MSGS_COMMON_H

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <Eigen/Geometry>
#include <boost/algorithm/clamp.hpp>

namespace mav_msgs {

const double kSmallValueCheck = 1.e-6;
const double kNumNanosecondsPerSecond = 1.e9;

/// Magnitude of Earth's gravitational field at specific height [m] and latitude
/// [rad] (from wikipedia).
inline double MagnitudeOfGravity(const double height,
                                 const double latitude_radians) {
  // gravity calculation constants
  const double kGravity_0 = 9.780327;
  const double kGravity_a = 0.0053024;
  const double kGravity_b = 0.0000058;
  const double kGravity_c = 3.155 * 1e-7;

  double sin_squared_latitude = std::sin(latitude_radians) * std::sin(latitude_radians);
  double sin_squared_twice_latitude =
      std::sin(2 * latitude_radians) * std::sin(2 * latitude_radians);
  return kGravity_0 * ((1 + kGravity_a * sin_squared_latitude -
                        kGravity_b * sin_squared_twice_latitude) -
                       kGravity_c * height);
}

inline Eigen::Vector3d vector3FromMsg(const geometry_msgs::Vector3& msg) {
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

inline Eigen::Vector3d vector3FromPointMsg(const geometry_msgs::Point& msg) {
  return Eigen::Vector3d(msg.x, msg.y, msg.z);
}

inline Eigen::Quaterniond quaternionFromMsg(
    const geometry_msgs::Quaternion& msg) {
  // Make sure this always returns a valid Quaternion, even if the message was
  // uninitialized or only approximately set.
  Eigen::Quaterniond quaternion(msg.w, msg.x, msg.y, msg.z);
  if (quaternion.norm() < std::numeric_limits<double>::epsilon()) {
    quaternion.setIdentity();
  } else {
    quaternion.normalize();
  }
  return quaternion;
}

inline void vectorEigenToMsg(const Eigen::Vector3d& eigen,
                             geometry_msgs::Vector3* msg) {
  assert(msg != NULL);
  msg->x = eigen.x();
  msg->y = eigen.y();
  msg->z = eigen.z();
}

inline void pointEigenToMsg(const Eigen::Vector3d& eigen,
                            geometry_msgs::Point* msg) {
  assert(msg != NULL);
  msg->x = eigen.x();
  msg->y = eigen.y();
  msg->z = eigen.z();
}

inline void quaternionEigenToMsg(const Eigen::Quaterniond& eigen,
                                 geometry_msgs::Quaternion* msg) {
  assert(msg != NULL);
  msg->x = eigen.x();
  msg->y = eigen.y();
  msg->z = eigen.z();
  msg->w = eigen.w();
}

/**
 * \brief Extracts the yaw part from a quaternion, using RPY / euler (z-y'-z'')
 * angles.
 * RPY rotates about the fixed axes in the order x-y-z,
 * which is the same as euler angles in the order z-y'-x''.
 */
inline double yawFromQuaternion(const Eigen::Quaterniond& q) {
  return std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
               1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
}

inline Eigen::Quaterniond quaternionFromYaw(double yaw) {
  return Eigen::Quaterniond(Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()));
}

inline void setQuaternionMsgFromYaw(double yaw,
                                    geometry_msgs::Quaternion* msg) {
  assert(msg != NULL);
  Eigen::Quaterniond q_yaw = quaternionFromYaw(yaw);
  msg->x = q_yaw.x();
  msg->y = q_yaw.y();
  msg->z = q_yaw.z();
  msg->w = q_yaw.w();
}

inline void setAngularVelocityMsgFromYawRate(double yaw_rate,
                                             geometry_msgs::Vector3* msg) {
  assert(msg != NULL);
  msg->x = 0.0;
  msg->y = 0.0;
  msg->z = yaw_rate;
}

inline void getEulerAnglesFromQuaternion(const Eigen::Quaternion<double>& q,
                                         Eigen::Vector3d* euler_angles) {
  {
    assert(euler_angles != NULL);

    *euler_angles << std::atan2(2.0 * (q.w() * q.x() + q.y() * q.z()),
                           1.0 - 2.0 * (q.x() * q.x() + q.y() * q.y())),
        std::asin(2.0 * (q.w() * q.y() - q.z() * q.x())),
        std::atan2(2.0 * (q.w() * q.z() + q.x() * q.y()),
              1.0 - 2.0 * (q.y() * q.y() + q.z() * q.z()));
  }
}

inline void skewMatrixFromVector(const Eigen::Vector3d& vec,
                                 Eigen::Matrix3d* vec_skew) {
  assert(vec_skew);
  *vec_skew << 0.0f, -vec(2), vec(1), vec(2), 0.0f, -vec(0), -vec(1), vec(0),
      0.0f;
}

inline bool vectorFromSkewMatrix(const Eigen::Matrix3d& vec_skew,
                                 Eigen::Vector3d* vec) {
  assert(vec);
  if ((vec_skew + vec_skew.transpose()).norm() < kSmallValueCheck){
    *vec << vec_skew(2,1), vec_skew(0,2), vec_skew(1,0);
    return true;
  } else {
    std::cerr << "[mav_msgs] Matrix is not skew-symmetric." << std::endl;
    *vec = Eigen::Vector3d::Zero();
    return false;
  }
}

inline bool isRotationMatrix(const Eigen::Matrix3d& mat){
  // Check that R^T * R = I
  if ((mat.transpose() * mat - Eigen::Matrix3d::Identity()).norm() > kSmallValueCheck){
    std::cerr << "[mav_msgs::common] Rotation matrix requirement violated (R^T * R = I)" << std::endl;
    return false;
  }  
  // Check that det(R) = 1
  if (mat.determinant() - 1.0 > kSmallValueCheck){
    std::cerr << "[mav_msgs::common] Rotation matrix requirement violated (det(R) = 1)" << std::endl;
    return false;
  }  
  return true;
}

// Rotation matrix from rotation vector as described in 
// "Computationally Efficient Trajectory Generation for Fully Actuated Multirotor Vehicles"
// Brescianini 2018
inline void matrixFromRotationVector(const Eigen::Vector3d& vec,
                                     Eigen::Matrix3d* mat) {
  // R = (I + sin||r|| / ||r||) [r] + ((1 - cos||r||)/||r||^2) [r]^2
  // where [r] is the skew matrix of r vector
  assert(mat);
  double r_norm = vec.norm();
  Eigen::Matrix3d vec_skew_norm = Eigen::Matrix3d::Zero();
  if (r_norm > 0.0){
    skewMatrixFromVector(vec / r_norm, &vec_skew_norm);
  }

  *mat = Eigen::Matrix3d::Identity() + vec_skew_norm * std::sin(r_norm) +
         vec_skew_norm * vec_skew_norm * (1 - std::cos(r_norm));
}

// Rotation vector from rotation matrix as described in 
// "Computationally Efficient Trajectory Generation for Fully Actuated Multirotor Vehicles"
// Brescianini 2018
inline bool vectorFromRotationMatrix(const Eigen::Matrix3d& mat,
                                     Eigen::Vector3d* vec) {
  // [r] = phi / 2sin(phi) * (R - R^T)
  // where [r] is the skew matrix of r vector
  // and phi satisfies 1 + 2cos(phi) = trace(R)
  assert(vec);
  
  if (!isRotationMatrix(mat)){
    std::cerr << "[mav_msgs::common] Not a rotation matrix." << std::endl;
    return false;
  }
  
  if ((mat - Eigen::Matrix3d::Identity()).norm() < kSmallValueCheck){
    *vec = Eigen::Vector3d::Zero();
    return true;
  }
  
  // Compute cosine of angle and clamp in range [-1, 1]
  double cos_phi = (mat.trace() - 1.0) / 2.0;
  double cos_phi_clamped = boost::algorithm::clamp(cos_phi, -1.0, 1.0);
  double phi = std::acos(cos_phi_clamped);
  
  if (phi < kSmallValueCheck){
    *vec = Eigen::Vector3d::Zero();
  } else{
    Eigen::Matrix3d vec_skew = (mat - mat.transpose()) * phi / (2.0 * std::sin(phi));
    Eigen::Vector3d vec_unskewed;
    if (vectorFromSkewMatrix(vec_skew, &vec_unskewed)){
      *vec = vec_unskewed;
    }else{
      return false;
    }
    
  }
  return true;
}

// Calculates angular velocity (omega) from rotation vector derivative
// based on formula derived in "Finite rotations and angular velocity" by Asher
// Peres
inline Eigen::Vector3d omegaFromRotationVector(
    const Eigen::Vector3d& rot_vec, const Eigen::Vector3d& rot_vec_vel) 
{
  double phi = rot_vec.norm();
  if (std::abs(phi) < 1.0e-3) {
    // This captures the case of zero rotation
    return rot_vec_vel;
  }

  double phi_inv = 1.0 / phi;
  double phi_2_inv = phi_inv / phi;
  double phi_3_inv = phi_2_inv / phi;

  // Create skew-symmetric matrix from rotation vector
  Eigen::Matrix3d phi_skew;
  skewMatrixFromVector(rot_vec, &phi_skew);

  // Set up matrix to calculate omega
  Eigen::Matrix3d W;
  W = Eigen::MatrixXd::Identity(3, 3) + phi_skew * (1 - std::cos(phi)) * phi_2_inv +
      phi_skew * phi_skew * (phi - std::sin(phi)) * phi_3_inv;
  return W * rot_vec_vel;
}

// Calculates angular acceleration (omegaDot) from rotation vector derivative
// based on formula derived in "Finite rotations and angular velocity" by Asher
// Peres
inline Eigen::Vector3d omegaDotFromRotationVector(
    const Eigen::Vector3d& rot_vec, const Eigen::Vector3d& rot_vec_vel,
    const Eigen::Vector3d& rot_vec_acc) 
{
  double phi = rot_vec.norm();
  if (std::abs(phi) < 1.0e-3) {
    // This captures the case of zero rotation
    return rot_vec_acc;
  }

  double phi_dot = rot_vec.dot(rot_vec_vel) / phi;

  double phi_inv = 1.0 / phi;
  double phi_2_inv = phi_inv / phi;
  double phi_3_inv = phi_2_inv / phi;
  double phi_4_inv = phi_3_inv / phi;


  // Create skew-symmetric matrix from rotation vector and velocity
  Eigen::Matrix3d phi_skew;
  Eigen::Matrix3d phi_dot_skew;

  skewMatrixFromVector(rot_vec, &phi_skew);
  skewMatrixFromVector(rot_vec_vel, &phi_dot_skew);

  // Set up matrices to calculate omega dot
  Eigen::Matrix3d W_vel;
  Eigen::Matrix3d W_acc;
  W_vel = phi_skew * (phi * std::sin(phi) - 2.0f + 2.0f * std::cos(phi)) * phi_dot *
              phi_3_inv +
          phi_skew * phi_skew *
              (-2.0f * phi - phi * std::cos(phi) + 3.0f * std::sin(phi)) * phi_dot *
              phi_4_inv +
          phi_dot_skew * phi_skew * (phi - std::sin(phi)) * phi_3_inv;

  W_acc = Eigen::MatrixXd::Identity(3, 3) +
          phi_skew * (1.0f - std::cos(phi)) * phi_2_inv +
          phi_skew * phi_skew * (phi - std::sin(phi)) * phi_3_inv;

  return W_vel * rot_vec_vel + W_acc * rot_vec_acc;
}

// Calculate the nominal rotor rates given the MAV mass, allocation matrix,
// angular velocity, angular acceleration, and body acceleration (normalized
// thrust).
//
// [torques, thrust]' = A * n^2, where
// torques = J * ang_acc + ang_vel x J
// thrust = m * norm(acc)
//
// The allocation matrix A has of a hexacopter is:
// A = K * B, where
// K = diag(l*c_T, l*c_T, c_M, c_T),
//     [ s  1  s -s -1 -s]
// B = [-c  0  c  c  0 -c]
//     [-1  1 -1  1 -1  1]
//     [ 1  1  1  1  1  1],
// l: arm length
// c_T: thrust constant
// c_M: moment constant
// s: sin(30°)
// c: cos(30°)
//
// The inverse can be computed computationally efficient:
// A^-1 \approx B^pseudo * K^-1
inline void getSquaredRotorSpeedsFromAllocationAndState(
    const Eigen::MatrixXd& allocation_inv, const Eigen::Vector3d& inertia,
    double mass, const Eigen::Vector3d& angular_velocity_B,
    const Eigen::Vector3d& angular_acceleration_B,
    const Eigen::Vector3d& acceleration_B,
    Eigen::VectorXd* rotor_rates_squared) {
  const Eigen::Vector3d torque =
      inertia.asDiagonal() * angular_acceleration_B +
      angular_velocity_B.cross(inertia.asDiagonal() * angular_velocity_B);
  const double thrust_force = mass * acceleration_B.norm();
  Eigen::Vector4d input;
  input << torque, thrust_force;
  *rotor_rates_squared = allocation_inv * input;
}

inline double nanosecondsToSeconds(int64_t nanoseconds) {
  double seconds = nanoseconds / kNumNanosecondsPerSecond;
  return seconds;
}

inline int64_t secondsToNanoseconds(double seconds) {
  int64_t nanoseconds =
      static_cast<int64_t>(seconds * kNumNanosecondsPerSecond);
  return nanoseconds;
}

}  // namespace mav_msgs

#endif  // MAV_MSGS_COMMON_H
