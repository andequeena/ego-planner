#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <nodelet/nodelet.h>
#include <quadrotor_msgs/Corrections.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/SO3Command.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <so3_control/SO3Control.h>
#include <std_msgs/Bool.h>
#include <tf/transform_datatypes.h>

// 这个类是 so3_control 的 ROS nodelet 包装层。
// 真正的控制算法在 SO3Control 中；Nodelet 负责订阅 odom/position_cmd/imu，
// 把消息转换成 Eigen 数据，调用控制器，并发布 quadrotor_msgs::SO3Command。
class SO3ControlNodelet : public nodelet::Nodelet
{
public:
  SO3ControlNodelet()
    : position_cmd_updated_(false)
    , position_cmd_init_(false)
    , des_yaw_(0)
    , des_yaw_dot_(0)
    , current_yaw_(0)
    , enable_motors_(true)
    , // FIXME
    use_external_yaw_(false)
  {
  }

  // nodelet 被 nodelet manager 加载后会自动调用 onInit()。
  // 它相当于普通 ROS node 里的 main 初始化部分。
  void onInit(void);

  // 类中有 Eigen::Vector3d 成员，使用该宏保证动态分配时满足 Eigen 对齐要求。
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // 根据当前期望轨迹和当前状态调用 SO3Control，并发布 so3_cmd。
  void publishSO3Command(void);
  // 接收上层规划器/示例节点发布的位置、速度、加速度、yaw 期望。
  void position_cmd_callback(
    const quadrotor_msgs::PositionCommand::ConstPtr& cmd);
  // 接收当前里程计，更新控制器里的当前位置和速度。
  void odom_callback(const nav_msgs::Odometry::ConstPtr& odom);
  // 接收电机使能/关闭开关。
  void enable_motors_callback(const std_msgs::Bool::ConstPtr& msg);
  // 接收推力或姿态修正量，常用于仿真/标定补偿。
  void corrections_callback(const quadrotor_msgs::Corrections::ConstPtr& msg);
  // 接收 IMU 加速度，给加速度反馈项使用。
  void imu_callback(const sensor_msgs::Imu& imu);

  // controller_ 是算法核心；其余 pub/sub 是 ROS 通信接口。
  SO3Control      controller_;
  ros::Publisher  so3_command_pub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber position_cmd_sub_;
  ros::Subscriber enable_motors_sub_;
  ros::Subscriber corrections_sub_;
  ros::Subscriber imu_sub_;

  // position_cmd_updated_ 用来判断两次 odom 之间是否收到了新的 position_cmd。
  // position_cmd_init_ 表示是否已经收到过至少一条 position_cmd。
  bool        position_cmd_updated_, position_cmd_init_;
  // 发布 SO3Command 时使用的坐标系名，通常是 "/quadrotor"。
  std::string frame_id_;

  // 期望状态和控制增益，由 position_cmd 或参数更新。
  Eigen::Vector3d des_pos_, des_vel_, des_acc_, kx_, kv_;
  double          des_yaw_, des_yaw_dot_;
  // 当前 yaw 从 odom 姿态中提取，随 SO3Command 的 aux 字段发布。
  double          current_yaw_;
  // enable_motors_ 控制是否允许电机输出；use_external_yaw_ 表示是否让下游使用外部 yaw。
  bool            enable_motors_;
  bool            use_external_yaw_;
  // kR_/kOm_ 是姿态环和角速度环增益；corrections_ 保存推力/角度修正。
  double          kR_[3], kOm_[3], corrections_[3];
  // 如果还没收到 position_cmd，可以用 init_x/y/z 作为初始悬停点。
  double          init_x_, init_y_, init_z_;
};

void
SO3ControlNodelet::publishSO3Command(void)
{
  // 用当前期望位置/速度/加速度和当前状态计算期望总力与期望姿态。
  controller_.calculateControl(des_pos_, des_vel_, des_acc_, des_yaw_,
                               des_yaw_dot_, kx_, kv_);

  // 从控制器取出计算结果，准备打包成 ROS 消息。
  const Eigen::Vector3d&    force       = controller_.getComputedForce();
  const Eigen::Quaterniond& orientation = controller_.getComputedOrientation();

  // SO3Command 发送给下游姿态/动力学控制或仿真模块。
  quadrotor_msgs::SO3Command::Ptr so3_command(
    new quadrotor_msgs::SO3Command); //! @note memory leak?
  so3_command->header.stamp    = ros::Time::now();
  so3_command->header.frame_id = frame_id_;
  so3_command->force.x         = force(0);
  so3_command->force.y         = force(1);
  so3_command->force.z         = force(2);
  so3_command->orientation.x   = orientation.x();
  so3_command->orientation.y   = orientation.y();
  so3_command->orientation.z   = orientation.z();
  so3_command->orientation.w   = orientation.w();
  for (int i = 0; i < 3; i++)
  {
    // 姿态控制器增益随消息一起下发。
    so3_command->kR[i]  = kR_[i];
    so3_command->kOm[i] = kOm_[i];
  }
  // aux 字段携带额外控制信息，例如当前 yaw、修正量、电机使能状态等。
  so3_command->aux.current_yaw          = current_yaw_;
  so3_command->aux.kf_correction        = corrections_[0];
  so3_command->aux.angle_corrections[0] = corrections_[1];
  so3_command->aux.angle_corrections[1] = corrections_[2];
  so3_command->aux.enable_motors        = enable_motors_;
  so3_command->aux.use_external_yaw     = use_external_yaw_;
  so3_command_pub_.publish(so3_command);
}

void
SO3ControlNodelet::position_cmd_callback(
  const quadrotor_msgs::PositionCommand::ConstPtr& cmd)
{
  // position_cmd 是上层规划/控制给出的期望状态。
  // 注意 SO3Control 中支持用 NaN 禁用某些低阶控制量，例如只做速度控制时 position 可设为 NaN。
  des_pos_ = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
  des_vel_ = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
  des_acc_ = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y,
                             cmd->acceleration.z);

  // 如果消息中给了非零 kx/kv，就用消息里的增益覆盖参数服务器中的默认增益。
  if ( cmd->kx[0] > 1e-5 || cmd->kx[1] > 1e-5 || cmd->kx[2] > 1e-5 )
  {
    kx_ = Eigen::Vector3d(cmd->kx[0], cmd->kx[1], cmd->kx[2]);
  }
  if ( cmd->kv[0] > 1e-5 || cmd->kv[1] > 1e-5 || cmd->kv[2] > 1e-5 )
  {
    kv_ = Eigen::Vector3d(cmd->kv[0], cmd->kv[1], cmd->kv[2]);
  }

  des_yaw_              = cmd->yaw;
  des_yaw_dot_          = cmd->yaw_dot;
  // 标记本轮已经收到新的 position_cmd；odom 回调会用这个标记决定是否需要补发控制命令。
  position_cmd_updated_ = true;
  position_cmd_init_    = true;

  // 一收到新的期望状态就立即发布一条控制命令，降低控制延迟。
  publishSO3Command();
}

void
SO3ControlNodelet::odom_callback(const nav_msgs::Odometry::ConstPtr& odom)
{
  // 从 odom 中提取当前位置和线速度，转换为 Eigen 向量。
  const Eigen::Vector3d position(odom->pose.pose.position.x,
                                 odom->pose.pose.position.y,
                                 odom->pose.pose.position.z);
  const Eigen::Vector3d velocity(odom->twist.twist.linear.x,
                                 odom->twist.twist.linear.y,
                                 odom->twist.twist.linear.z);

  // 从四元数姿态中取出当前 yaw，作为辅助信息下发。
  current_yaw_ = tf::getYaw(odom->pose.pose.orientation);

  // 更新控制器内部当前状态，下一次 calculateControl 会使用这些状态计算误差。
  controller_.setPosition(position);
  controller_.setVelocity(velocity);

  if (position_cmd_init_)
  {
    // We set position_cmd_updated_ = false and expect that the
    // position_cmd_callback would set it to true since typically a position_cmd
    // message would follow an odom message. If not, the position_cmd_callback
    // hasn't been called and we publish the so3 command ourselves
    // TODO: Fallback to hover if position_cmd hasn't been received for some
    // time
    // 如果这一轮 odom 到来时没有新的 position_cmd，也用上一条期望状态重新算一次控制命令。
    // 这样即使期望轨迹发布频率略低于 odom，也能持续输出控制。
    if (!position_cmd_updated_)
      publishSO3Command();
    // 清零标记，等待下一条 position_cmd_callback 再置 true。
    position_cmd_updated_ = false;
  }
  else if ( init_z_ > -9999.0 )
  {
    // 尚未收到 position_cmd 时，如果参数里配置了初始悬停点，则先飞/停到该位置。
    des_pos_ = Eigen::Vector3d(init_x_, init_y_, init_z_);
    des_vel_ = Eigen::Vector3d(0,0,0);
    des_acc_ = Eigen::Vector3d(0,0,0);
    publishSO3Command();
  }
  
}

void
SO3ControlNodelet::enable_motors_callback(const std_msgs::Bool::ConstPtr& msg)
{
  // motors topic 用于开关电机输出，状态会放进 SO3Command 的 aux.enable_motors。
  if (msg->data)
    ROS_INFO("Enabling motors");
  else
    ROS_INFO("Disabling motors");

  enable_motors_ = msg->data;
}

void
SO3ControlNodelet::corrections_callback(
  const quadrotor_msgs::Corrections::ConstPtr& msg)
{
  // 保存外部修正量，后面发布 SO3Command 时透传给下游控制器。
  corrections_[0] = msg->kf_correction;
  corrections_[1] = msg->angle_corrections[0];
  corrections_[2] = msg->angle_corrections[1];
}

void
SO3ControlNodelet::imu_callback(const sensor_msgs::Imu& imu)
{
  // IMU 线加速度用于 SO3Control 中的加速度误差反馈项。
  const Eigen::Vector3d acc(imu.linear_acceleration.x,
                            imu.linear_acceleration.y,
                            imu.linear_acceleration.z);
  controller_.setAcc(acc);
}

void
SO3ControlNodelet::onInit(void)
{
  // nodelet 使用 getPrivateNodeHandle() 获取私有命名空间，
  // 因此下面的参数和 topic 名通常相对于该 nodelet 的私有 namespace。
  ros::NodeHandle n(getPrivateNodeHandle());

  std::string quadrotor_name;
  n.param("quadrotor_name", quadrotor_name, std::string("quadrotor"));
  frame_id_ = "/" + quadrotor_name;

  double mass;
  // 从参数服务器读取质量并设置到控制器。
  n.param("mass", mass, 0.5);
  controller_.setMass(mass);

  // 是否使用外部 yaw，由 SO3Command 的 aux.use_external_yaw 传给下游。
  n.param("use_external_yaw", use_external_yaw_, true);

  // 读取姿态环/角速度环/位置速度环增益。
  n.param("gains/rot/x", kR_[0], 1.5);
  n.param("gains/rot/y", kR_[1], 1.5);
  n.param("gains/rot/z", kR_[2], 1.0);
  n.param("gains/ang/x", kOm_[0], 0.13);
  n.param("gains/ang/y", kOm_[1], 0.13);
  n.param("gains/ang/z", kOm_[2], 0.1);
  n.param("gains/kx/x", kx_[0], 5.7);
  n.param("gains/kx/y", kx_[1], 5.7);
  n.param("gains/kx/z", kx_[2], 6.2);
  n.param("gains/kv/x", kv_[0], 3.4);
  n.param("gains/kv/y", kv_[1], 3.4);
  n.param("gains/kv/z", kv_[2], 4.0);

  // 读取推力和姿态角修正参数。
  n.param("corrections/z", corrections_[0], 0.0);
  n.param("corrections/r", corrections_[1], 0.0);
  n.param("corrections/p", corrections_[2], 0.0);

  // 初始悬停点。默认 init_z_ = -10000 表示不启用初始悬停逻辑。
  n.param("so3_control/init_state_x", init_x_, 0.0);
  n.param("so3_control/init_state_y", init_y_, 0.0);
  n.param("so3_control/init_state_z", init_z_, -10000.0);

  // 发布给下游控制/仿真模块的 SO3 控制命令。
  so3_command_pub_ = n.advertise<quadrotor_msgs::SO3Command>("so3_cmd", 10);

  // 订阅当前状态、期望状态和辅助控制 topic。
  // tcpNoDelay() 减少 TCPROS 的小包延迟，对控制回路更友好。
  odom_sub_ = n.subscribe("odom", 10, &SO3ControlNodelet::odom_callback, this,
                          ros::TransportHints().tcpNoDelay());
  position_cmd_sub_ =
    n.subscribe("position_cmd", 10, &SO3ControlNodelet::position_cmd_callback,
                this, ros::TransportHints().tcpNoDelay());

  enable_motors_sub_ =
    n.subscribe("motors", 2, &SO3ControlNodelet::enable_motors_callback, this,
                ros::TransportHints().tcpNoDelay());
  corrections_sub_ =
    n.subscribe("corrections", 10, &SO3ControlNodelet::corrections_callback,
                this, ros::TransportHints().tcpNoDelay());

  imu_sub_ = n.subscribe("imu", 10, &SO3ControlNodelet::imu_callback, this,
                         ros::TransportHints().tcpNoDelay());
}

#include <pluginlib/class_list_macros.h>
//PLUGINLIB_DECLARE_CLASS(so3_control, SO3ControlNodelet, SO3ControlNodelet,
//                        nodelet::Nodelet);
// 把 SO3ControlNodelet 注册为 pluginlib 插件，使 nodelet manager 可以按插件名动态加载它。
PLUGINLIB_EXPORT_CLASS(SO3ControlNodelet, nodelet::Nodelet);
