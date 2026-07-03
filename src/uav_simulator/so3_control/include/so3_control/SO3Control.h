#ifndef __SO3_CONTROL_H__
#define __SO3_CONTROL_H__

#include <Eigen/Geometry>

// SO3Control 是四旋翼的核心位置/速度/加速度控制器。
// 这里的 SO(3) 指三维旋转群，控制器最终输出的不是 roll/pitch/yaw 欧拉角，
// 而是一个期望姿态四元数 orientation_ 和总推力向量 force_。
class SO3Control
{
public:
  SO3Control();

  // 设置飞机质量，控制力计算中会用 mass_ * g_ 抵消重力，也会用质量把期望加速度转换成力。
  void setMass(const double mass);
  // 设置重力加速度，默认一般是 9.81。
  void setGravity(const double g);
  // 从里程计更新当前世界系位置。
  void setPosition(const Eigen::Vector3d& position);
  // 从里程计更新当前世界系速度。
  void setVelocity(const Eigen::Vector3d& velocity);
  // 从 IMU 更新当前线加速度，用于加速度前馈/反馈项。
  void setAcc(const Eigen::Vector3d& acc);

  // 根据期望位置/速度/加速度/yaw 和控制增益，计算期望力 force_ 与期望姿态 orientation_。
  // des_pos/des_vel/des_acc 中如果某一类命令被设置为 NaN，cpp 中会跳过对应控制项。
  // 例如只想做速度控制时，可以把 des_pos 设置为 NaN，让位置反馈不参与计算。
  void calculateControl(const Eigen::Vector3d& des_pos,
                        const Eigen::Vector3d& des_vel,
                        const Eigen::Vector3d& des_acc, const double des_yaw,
                        const double des_yaw_dot, const Eigen::Vector3d& kx,
                        const Eigen::Vector3d& kv);

  // 返回最近一次 calculateControl() 计算出的总推力向量。
  const Eigen::Vector3d&    getComputedForce(void);
  // 返回最近一次 calculateControl() 计算出的期望机体系姿态四元数。
  const Eigen::Quaterniond& getComputedOrientation(void);

  // Eigen 的固定大小向量/四元数在类里作为成员时需要内存对齐支持，
  // 这个宏可以避免 new 这个类时出现 Eigen 对齐相关问题。
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // Inputs for the controller
  // mass_: 飞机质量；g_: 重力加速度。
  double          mass_;
  double          g_;
  // pos_/vel_/acc_ 是当前状态，由 odom 和 imu 回调持续更新。
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  Eigen::Vector3d acc_;

  // Outputs of the controller
  // force_ 是世界系下需要产生的合力方向和大小。
  Eigen::Vector3d    force_;
  // orientation_ 是为了产生 force_ 同时满足期望 yaw 而构造出的期望姿态。
  Eigen::Quaterniond orientation_;
};

#endif
