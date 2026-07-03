#include <Eigen/Eigen>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/ros.h>

// 这个文件是 so3_control 的简单指令发布示例。
// 它不是控制器本体，而是周期性向 /position_cmd 发布不同类型的期望命令，
// 用来演示 SO3Control 支持位置控制、速度控制和加速度控制三种输入方式。
int main(int argc, char **argv)
{
  // 初始化 ROS 节点，节点名为 quad_sim_example。
  ros::init(argc, argv, "quad_sim_example");
  // 使用私有 NodeHandle。注意下面 advertise 使用的是绝对 topic "/position_cmd"，
  // 因此实际发布到全局 /position_cmd，而不是 "~position_cmd"。
  ros::NodeHandle nh("~");

  // 发布 quadrotor_msgs::PositionCommand，so3_control_nodelet 会订阅该消息并转换成 SO3Command。
  ros::Publisher cmd_pub = nh.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 10);

  // 等待一小段时间，让 publisher 和 subscriber 建立连接。
  ros::Duration(2.0).sleep();

  while (ros::ok())
  {

    /*** example 1: position control ***/
    // 示例 1：位置控制。
    // 只设置 position，速度和加速度保持消息默认值，控制器会把期望位置作为主要反馈目标。
    std::cout << "\033[42m"
              << "Position Control to (2,0,1) meters"
              << "\033[0m" << std::endl;
    for (int i = 0; i < 500; i++)
    {
      // 每 0.01 秒发布一次，500 次约等于持续 5 秒。
      quadrotor_msgs::PositionCommand cmd;
      cmd.position.x = 2.0;
      cmd.position.y = 0.0;
      cmd.position.z = 1.0;
      cmd_pub.publish(cmd);

      ros::Duration(0.01).sleep();
      ros::spinOnce();
    }

    /*** example 1: position control ***/
    // 示例 2：速度控制。
    // 这里故意把 position 设置为 NaN，表示“不使用位置控制项”。
    // SO3Control::calculateControl() 会检测 NaN，并跳过对应的 des_pos - pos_ 反馈。
    std::cout << "\033[42m"
              << "Velocity Control to (-1,0,0) meters/second"
              << "\033[0m" << std::endl;
    for (int i = 0; i < 500; i++)
    {
      quadrotor_msgs::PositionCommand cmd;
      // lower-order commands must be disabled by nan 的意思是：
      // 当想测试速度命令时，必须用 NaN 显式禁用位置命令，否则位置误差仍会参与控制。
      cmd.position.x = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.position.y = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.position.z = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.velocity.x = -1.0;
      cmd.velocity.y = 0.0;
      cmd.velocity.z = 0.0;
      cmd_pub.publish(cmd);

      ros::Duration(0.01).sleep();
      ros::spinOnce();
    }

    /*** example 1: accelleration control ***/
    // 示例 3：加速度控制。
    // 同时把 position 和 velocity 设置为 NaN，只留下 acceleration 作为有效指令。
    std::cout << "\033[42m"
              << "Accelleration Control to (1,0,0) meters/second^2"
              << "\033[0m" << std::endl;
    for (int i = 0; i < 500; i++)
    {
      quadrotor_msgs::PositionCommand cmd;
      // 禁用位置控制项。
      cmd.position.x = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.position.y = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      cmd.position.z = std::numeric_limits<float>::quiet_NaN(); // lower-order commands must be disabled by nan
      // 禁用速度控制项。
      cmd.velocity.x = std::numeric_limits<float>::quiet_NaN();
      cmd.velocity.y = std::numeric_limits<float>::quiet_NaN();
      cmd.velocity.z = std::numeric_limits<float>::quiet_NaN();
      // 只给 x 方向 1 m/s^2 的期望加速度。
      cmd.acceleration.x = 1.0;
      cmd.acceleration.y = 0.0;
      cmd.acceleration.z = 0.0;
      cmd_pub.publish(cmd);

      ros::Duration(0.01).sleep();
      ros::spinOnce();
    }

  }

  return 0;
}
