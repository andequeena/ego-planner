#include <iostream>
#include <so3_control/SO3Control.h>

#include <ros/ros.h>

SO3Control::SO3Control()
    : mass_(0.5), g_(9.81)
{
    // 默认当前加速度置零；位置和速度会在 odom/imu 回调中更新。
    acc_.setZero();
}

void SO3Control::setMass(const double mass)
{
    // 质量会影响重力补偿项 mass*g 和加速度前馈项 mass*des_acc。
    mass_ = mass;
}

void SO3Control::setGravity(const double g)
{
    // 允许外部修改重力常数，仿真或不同环境下可配置。
    g_ = g;
}

void SO3Control::setPosition(const Eigen::Vector3d &position)
{
    // 当前世界系位置，通常来自 odometry。
    pos_ = position;
}

void SO3Control::setVelocity(const Eigen::Vector3d &velocity)
{
    // 当前世界系速度，通常来自 odometry。
    vel_ = velocity;
}

void SO3Control::calculateControl(const Eigen::Vector3d &des_pos,
                                  const Eigen::Vector3d &des_vel,
                                  const Eigen::Vector3d &des_acc,
                                  const double des_yaw, const double des_yaw_dot,
                                  const Eigen::Vector3d &kx,
                                  const Eigen::Vector3d &kv)
{
    //  ROS_INFO("Error %lf %lf %lf", (des_pos - pos_).norm(),
    //           (des_vel - vel_).norm(), (des_acc - acc_).norm());

    // 这里用 NaN 作为“禁用某一类控制量”的标记。
    // 例如 des_pos 里任一维是 NaN，就认为本次不做位置反馈，只使用速度/加速度等其它可用项。
    bool flag_use_pos = !(std::isnan(des_pos(0)) || std::isnan(des_pos(1)) || std::isnan(des_pos(2)));
    bool flag_use_vel = !(std::isnan(des_vel(0)) || std::isnan(des_vel(1)) || std::isnan(des_vel(2)));
    bool flag_use_acc = !(std::isnan(des_acc(0)) || std::isnan(des_acc(1)) || std::isnan(des_acc(2)));

    // totalError 用于估计整体误差大小，后面根据误差大小调整加速度反馈增益 ka。
    // 只有没有被 NaN 禁用的指令项才参与误差统计。
    Eigen::Vector3d totalError(Eigen::Vector3d::Zero());
    if (flag_use_pos)
        // 左边 totalError 和右边 des_pos - pos_ 没有内存重叠，可以直接高效计算
        totalError.noalias() += des_pos - pos_;
    if (flag_use_vel)
        totalError.noalias() += des_vel - vel_;
    if (flag_use_acc)
        totalError.noalias() += des_acc - acc_;

    // ka是加速度误差的自适应增益：
    // 误差绝对值超过 3 时设为 0，避免加速度反馈过猛；
    // 误差较小时按误差大小线性增加，用于温和地修正实际加速度。范围是[0,0.6]
    Eigen::Vector3d ka(fabs(totalError[0]) > 3 ? 0 : (fabs(totalError[0]) * 0.2),
                       fabs(totalError[1]) > 3 ? 0 : (fabs(totalError[1]) * 0.2),
                       fabs(totalError[2]) > 3 ? 0 : (fabs(totalError[2]) * 0.2));

    // std::cout << des_pos.transpose() << std::endl;
    // std::cout << des_vel.transpose() << std::endl;
    // std::cout << des_acc.transpose() << std::endl;
    // std::cout << des_yaw << std::endl;
    // std::cout << pos_.transpose() << std::endl;
    // std::cout << vel_.transpose() << std::endl;
    // std::cout << acc_.transpose() << std::endl;

    // 先加入重力补偿：如果没有任何期望指令，至少需要 mass*g 向上的力来悬停。
    force_ = mass_ * g_ * Eigen::Vector3d(0, 0, 1);
    // 位置反馈项：kx 是 xyz 三个方向的位置增益，对应 kx * (期望位置 - 当前位置)。
    if (flag_use_pos)
        force_.noalias() += kx.asDiagonal() * (des_pos - pos_);
    // 速度反馈项：kv 是 xyz 三个方向的速度增益，对应 kv * (期望速度 - 当前速度)。
    if (flag_use_vel)
        force_.noalias() += kv.asDiagonal() * (des_vel - vel_);
    // 加速度项由两部分组成：
    // 1. mass * des_acc 是期望加速度前馈；
    // 2. mass * ka * (des_acc - acc_) 是实际加速度误差反馈。
    if (flag_use_acc)
        force_.noalias() += mass_ * ka.asDiagonal() * (des_acc - acc_) + mass_ * (des_acc);

    // Limit control angle to 45 degree
    double theta = M_PI / 2;
    double c = cos(theta);
    Eigen::Vector3d f;
    // f 去掉了重力补偿，只表示横向/额外控制力。
    f.noalias() = force_ - mass_ * g_ * Eigen::Vector3d(0, 0, 1);
    // 判断总推力方向和世界 z 轴夹角是否超过限制。
    // 如果夹角过大，就缩放额外控制力 f，使最终 force_ 不要求飞机倾斜得太厉害。
    if (Eigen::Vector3d(0, 0, 1).dot(force_ / force_.norm()) < c)
    {
        // 这里通过解一个二次方程求缩放系数 s，使 s*f + mg*z 的方向刚好满足倾角约束。
        double nf = f.norm();
        double A = c * c * nf * nf - f(2) * f(2);
        double B = 2 * (c * c - 1) * f(2) * mass_ * g_;
        double C = (c * c - 1) * mass_ * mass_ * g_ * g_;
        double s = (-B + sqrt(B * B - 4 * A * C)) / (2 * A);
        force_.noalias() = s * f + mass_ * g_ * Eigen::Vector3d(0, 0, 1);
    }
    // Limit control angle to 45 degree

    Eigen::Vector3d b1c, b2c, b3c;
    // b1d 是期望 yaw 在水平面上的朝向，即机体系 x 轴希望投影到的方向。
    Eigen::Vector3d b1d(cos(des_yaw), sin(des_yaw), 0);

    // b3c 是期望机体系 z 轴方向。
    // 四旋翼主要沿机体系 z 轴产生推力，因此 b3c 要和期望总力 force_ 对齐。
    if (force_.norm() > 1e-6)
        b3c.noalias() = force_.normalized();
    else
        b3c.noalias() = Eigen::Vector3d(0, 0, 1);

    // 用 b3c 和期望 yaw 方向 b1d 构造一个正交坐标系：
    // b2c = b3c x b1d，随后 b1c = b2c x b3c。
    // 最终 [b1c b2c b3c] 就是期望姿态旋转矩阵的三个列向量。
    b2c.noalias() = b3c.cross(b1d).normalized();
    b1c.noalias() = b2c.cross(b3c).normalized();

    Eigen::Matrix3d R;
    R << b1c, b2c, b3c;

    // 将旋转矩阵转换成四元数，后续发布给 SO3 控制/仿真模块。
    orientation_ = Eigen::Quaterniond(R);
}

const Eigen::Vector3d &
SO3Control::getComputedForce(void)
{
    // 返回上一次控制计算结果；调用者不要修改这个引用。
    return force_;
}

const Eigen::Quaterniond &
SO3Control::getComputedOrientation(void)
{
    // 返回上一次控制计算得到的期望姿态。
    return orientation_;
}

void SO3Control::setAcc(const Eigen::Vector3d &acc)
{
    // 当前加速度一般来自 IMU 线加速度，用于加速度误差反馈。
    acc_ = acc;
}
