#include "bspline_opt/uniform_bspline.h"
#include "nav_msgs/Odometry.h"
#include "ego_planner/Bspline.h"
#include "quadrotor_msgs/PositionCommand.h"
#include "std_msgs/Empty.h"
#include "visualization_msgs/Marker.h"
#include <ros/ros.h>

// 发布给控制器的期望位置/速度/加速度/yaw 指令。
// 下游控制节点通常订阅 /position_cmd，并按照这个指令跟踪轨迹。
ros::Publisher pos_cmd_pub;

// 复用同一个 PositionCommand 消息对象，每次定时器回调时更新其中的字段并发布。
quadrotor_msgs::PositionCommand cmd;

// 位置环和速度环增益，会写入 cmd.kx / cmd.kv。
// 这里默认全为 0，表示 traj_server 只负责发布参考轨迹；
// 实际控制增益通常由下游控制器或其他参数文件决定。
double pos_gain[3] = {0, 0, 0};
double vel_gain[3] = {0, 0, 0};

using ego_planner::UniformBspline;

// 是否已经收到规划器发布的 B 样条轨迹。
// 在收到轨迹之前，定时器回调不会发布位置指令。
bool receive_traj_ = false;

// traj_ 中保存当前正在执行的轨迹：
//   traj_[0]：位置 B 样条 p(t)
//   traj_[1]：速度 B 样条 v(t)，由位置轨迹求一阶导得到
//   traj_[2]：加速度 B 样条 a(t)，由速度轨迹再求一阶导得到
vector<UniformBspline> traj_;

// 当前轨迹总时长，由位置 B 样条的 knot span 计算得到。
double traj_duration_;

// 当前轨迹的起始执行时间。t_cur = now - start_time_。
ros::Time start_time_;

// 当前轨迹编号，原样转发到 PositionCommand，方便下游区分不同轨迹。
int traj_id_;

// yaw control
// 保存上一时刻发布的 yaw 和 yaw_dot，用于连续限速和低通滤波。
double last_yaw_, last_yaw_dot_;

// yaw 前视时间：计算朝向时不看当前位置，而是看 t_cur + time_forward_ 的前方轨迹点。
// 这样无人机机头会提前朝向未来运动方向，而不是只跟随瞬时速度抖动。
double time_forward_;

void bsplineCallback(ego_planner::BsplineConstPtr msg)
{
    // parse pos traj

    // 从 Bspline 消息中读取位置控制点。
    // pos_pts 的矩阵形状是 3 x N，每一列是一个三维控制点。
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());

    // knot vector 是 B 样条的节点向量，决定每段样条对应的时间区间。
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t i = 0; i < msg->knots.size(); ++i)
    {
        knots(i) = msg->knots[i];
    }

    // ROS 消息里的 pos_pts 是 geometry_msgs/Point 数组；
    // 这里转换成 Eigen 矩阵，供 UniformBspline 类使用。
    for (size_t i = 0; i < msg->pos_pts.size(); ++i)
    {
        pos_pts(0, i) = msg->pos_pts[i].x;
        pos_pts(1, i) = msg->pos_pts[i].y;
        pos_pts(2, i) = msg->pos_pts[i].z;
    }

    // 先用控制点、阶数和一个占位 interval 构造位置 B 样条。
    // 后面马上 setKnot(knots)，因此真实时间分配由消息中的 knot vector 决定。
    UniformBspline pos_traj(pos_pts, msg->order, 0.1);
    pos_traj.setKnot(knots);

    // parse yaw traj

    // Eigen::MatrixXd yaw_pts(msg->yaw_pts.size(), 1);
    // for (int i = 0; i < msg->yaw_pts.size(); ++i) {
    //   yaw_pts(i, 0) = msg->yaw_pts[i];
    // }

    // UniformBspline yaw_traj(yaw_pts, msg->order, msg->yaw_dt);

    // 记录这条轨迹的开始执行时间和编号。
    // 后续定时器按照 ROS 当前时间计算 t_cur，从而在 B 样条上取对应状态。
    start_time_ = msg->start_time;
    traj_id_ = msg->traj_id;

    // 保存位置轨迹，并自动构造速度、加速度轨迹。
    // getDerivative() 对 B 样条求导，得到同样可用 De Boor 算法查询的导数样条。
    traj_.clear();
    traj_.push_back(pos_traj);
    traj_.push_back(traj_[0].getDerivative());
    traj_.push_back(traj_[1].getDerivative());

    // 当前轨迹总时长，用于判断轨迹是否执行结束。
    traj_duration_ = traj_[0].getTimeSum();

    // 标记已经收到有效轨迹，cmdCallback 可以开始发布控制指令。
    receive_traj_ = true;
}

std::pair<double, double> calculate_yaw(double t_cur, Eigen::Vector3d &pos, ros::Time &time_now, ros::Time &time_last)
{
    // yaw 角统一限制在 [-pi, pi] 附近处理。
    constexpr double PI = 3.1415926;

    // 最大 yaw 角速度，这里设置为 pi rad/s，即每秒最多转 180 度。
    constexpr double YAW_DOT_MAX_PER_SEC = PI;
    // constexpr double YAW_DOT_DOT_MAX_PER_SEC = PI;

    // 返回值 first 是 yaw，second 是 yaw_dot。
    std::pair<double, double> yaw_yawdot(0, 0);
    double yaw = 0;
    double yawdot = 0;

    // 根据前视点计算期望机头朝向：
    //   如果 t_cur + time_forward_ 没超过轨迹总时长，就看前方 time_forward_ 秒的位置；
    //   否则看轨迹终点。
    // dir 表示“当前位置 pos 指向前视点”的方向向量。
    Eigen::Vector3d dir = t_cur + time_forward_ <= traj_duration_ ? traj_[0].evaluateDeBoorT(t_cur + time_forward_) - pos : traj_[0].evaluateDeBoorT(traj_duration_) - pos;

    // 如果前视方向足够明显，就用 atan2 得到期望 yaw；
    // 如果 dir 太短，说明当前位置和前视点几乎重合，此时保持上一帧 yaw，避免角度噪声抖动。
    double yaw_temp = dir.norm() > 0.1 ? atan2(dir(1), dir(0)) : last_yaw_;

    // 按两次定时器回调之间的实际时间，计算本周期允许的最大 yaw 改变量。
    double max_yaw_change = YAW_DOT_MAX_PER_SEC * (time_now - time_last).toSec();

    // 下面的分支主要处理 yaw 角跨越 -pi/pi 边界的问题。
    // 例如从 179 度转到 -179 度，实际只需要转 2 度，
    // 不能直接按数值差 -358 度去旋转。
    if (yaw_temp - last_yaw_ > PI)
    {
        // yaw_temp 数值上比 last_yaw_ 大超过 pi，说明可能跨过了 pi 到 -pi 的边界。
        // 这里尝试走“负方向短路径”，并用 max_yaw_change 限制单周期转角。
        if (yaw_temp - last_yaw_ - 2 * PI < -max_yaw_change)
        {
            yaw = last_yaw_ - max_yaw_change;
            if (yaw < -PI)
                yaw += 2 * PI;

            yawdot = -YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            // 如果目标 yaw 已在本周期可达范围内，就直接设置为 yaw_temp。
            yaw = yaw_temp;
            if (yaw - last_yaw_ > PI)
                yawdot = -YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }
    else if (yaw_temp - last_yaw_ < -PI)
    {
        // yaw_temp 数值上比 last_yaw_ 小超过 pi，说明可能从 -pi 跨到 pi。
        // 这里走“正方向短路径”，同样限制单周期最大转角。
        if (yaw_temp - last_yaw_ + 2 * PI > max_yaw_change)
        {
            yaw = last_yaw_ + max_yaw_change;
            if (yaw > PI)
                yaw -= 2 * PI;

            yawdot = YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            yaw = yaw_temp;
            if (yaw - last_yaw_ < -PI)
                yawdot = YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }
    else
    {
        // 普通情况：yaw_temp 和 last_yaw_ 没有跨越 pi 边界。
        // 若差值超过本周期允许变化量，就按最大角速度逐步逼近。
        if (yaw_temp - last_yaw_ < -max_yaw_change)
        {
            yaw = last_yaw_ - max_yaw_change;
            if (yaw < -PI)
                yaw += 2 * PI;

            yawdot = -YAW_DOT_MAX_PER_SEC;
        }
        else if (yaw_temp - last_yaw_ > max_yaw_change)
        {
            yaw = last_yaw_ + max_yaw_change;
            if (yaw > PI)
                yaw -= 2 * PI;

            yawdot = YAW_DOT_MAX_PER_SEC;
        }
        else
        {
            // 目标 yaw 在本周期允许范围内，可以直接到达。
            yaw = yaw_temp;
            if (yaw - last_yaw_ > PI)
                yawdot = -YAW_DOT_MAX_PER_SEC;
            else if (yaw - last_yaw_ < -PI)
                yawdot = YAW_DOT_MAX_PER_SEC;
            else
                yawdot = (yaw_temp - last_yaw_) / (time_now - time_last).toSec();
        }
    }

    // 对 yaw 做一个非常简单的一阶低通滤波，减少指令抖动。
    // 注：原注释里写作 nieve LPF，应是 naive LPF，保留原文不改。
    if (fabs(yaw - last_yaw_) <= max_yaw_change)
        yaw = 0.5 * last_yaw_ + 0.5 * yaw; // nieve LPF

    // yawdot 同样做低通滤波，使角速度指令更平滑。
    yawdot = 0.5 * last_yaw_dot_ + 0.5 * yawdot;

    // 更新历史 yaw 状态，供下一次回调继续做限速和滤波。
    last_yaw_ = yaw;
    last_yaw_dot_ = yawdot;

    // 打包返回 yaw 和 yaw_dot。
    yaw_yawdot.first = yaw;
    yaw_yawdot.second = yawdot;

    return yaw_yawdot;
}

void cmdCallback(const ros::TimerEvent &e)
{
    /* no publishing before receive traj_ */
    // 在收到第一条 B 样条轨迹之前，不发布控制指令。
    if (!receive_traj_)
        return;

    // 当前 ROS 时间，以及当前轨迹已经执行的相对时间 t_cur。
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - start_time_).toSec();

    // 本周期要发布的位置、速度、加速度。
    // pos_f 是前视位置变量，当前代码中只计算但没有写入 cmd。
    Eigen::Vector3d pos(Eigen::Vector3d::Zero()), vel(Eigen::Vector3d::Zero()), acc(Eigen::Vector3d::Zero()), pos_f;

    // 本周期要发布的 yaw 和 yaw_dot。
    std::pair<double, double> yaw_yawdot(0, 0);

    // 保存上一帧定时器时间，用于计算 yaw 限速时的真实 dt。
    static ros::Time time_last = ros::Time::now();

    // 轨迹正在执行中：从 B 样条上按当前时间 t_cur 查询位置、速度、加速度。
    if (t_cur < traj_duration_ && t_cur >= 0.0)
    {
        pos = traj_[0].evaluateDeBoorT(t_cur);
        vel = traj_[1].evaluateDeBoorT(t_cur);
        acc = traj_[2].evaluateDeBoorT(t_cur);

        /*** calculate yaw ***/
        yaw_yawdot = calculate_yaw(t_cur, pos, time_now, time_last);
        /*** calculate yaw ***/

        // 计算 2 秒后的前视位置，但目前没有实际发布或使用。
        // 这里可能是保留给调试、显示或后续控制策略的变量。
        double tf = min(traj_duration_, t_cur + 2.0);
        pos_f = traj_[0].evaluateDeBoorT(tf);
    }
    else if (t_cur >= traj_duration_)
    {
        /* hover when finish traj_ */
        // 轨迹执行结束后，在终点悬停：
        // 位置固定到轨迹末端，速度和加速度置零。
        pos = traj_[0].evaluateDeBoorT(traj_duration_);
        vel.setZero();
        acc.setZero();

        // yaw 保持上一帧，yaw_dot 置零，避免结束时继续旋转。
        yaw_yawdot.first = last_yaw_;
        yaw_yawdot.second = 0;

        pos_f = pos;
    }
    else
    {
        // t_cur < 0 通常说明当前时间早于轨迹 start_time_，
        // 例如时间同步问题或轨迹刚发布但 start_time 在未来。
        cout << "[Traj server]: invalid time." << endl;
    }

    // 更新上一帧时间，供下一次 yaw 计算使用。
    time_last = time_now;

    // 填充 PositionCommand 的公共头信息。
    cmd.header.stamp = time_now;
    cmd.header.frame_id = "world";

    // 标记轨迹指令已经准备好，下游控制器可以按该消息跟踪。
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd.trajectory_id = traj_id_;

    // 期望位置。
    cmd.position.x = pos(0);
    cmd.position.y = pos(1);
    cmd.position.z = pos(2);

    // 期望速度。
    cmd.velocity.x = vel(0);
    cmd.velocity.y = vel(1);
    cmd.velocity.z = vel(2);

    // 期望加速度，通常用于前馈控制。
    cmd.acceleration.x = acc(0);
    cmd.acceleration.y = acc(1);
    cmd.acceleration.z = acc(2);

    // 期望偏航角和偏航角速度。
    cmd.yaw = yaw_yawdot.first;
    cmd.yaw_dot = yaw_yawdot.second;

    // 再同步一次 last_yaw_，确保发布出去的 yaw 就是下一周期的历史 yaw。
    last_yaw_ = cmd.yaw;

    // 发布给控制器。
    pos_cmd_pub.publish(cmd);
}

int main(int argc, char **argv)
{
    // 初始化 ROS 节点，节点名为 traj_server。
    ros::init(argc, argv, "traj_server");

    // node 用于全局命名空间的话题订阅/发布；
    // nh("~") 用于读取私有参数，例如 ~traj_server/time_forward。
    ros::NodeHandle node;
    ros::NodeHandle nh("~");

    // 订阅规划器发布的 B 样条轨迹。
    // 收到消息后，bsplineCallback 会解析轨迹并更新 traj_。
    ros::Subscriber bspline_sub = node.subscribe("planning/bspline", 10, bsplineCallback);

    // 发布位置控制指令，下游控制器订阅 /position_cmd。
    pos_cmd_pub = node.advertise<quadrotor_msgs::PositionCommand>("/position_cmd", 50);

    // 100 Hz 定时发布 PositionCommand。
    // 每 0.01s 根据当前时间在 B 样条上取一次位置/速度/加速度。
    ros::Timer cmd_timer = node.createTimer(ros::Duration(0.01), cmdCallback);

    /* control parameter */
    // 把位置增益写入 PositionCommand。
    // 当前 pos_gain 默认全 0，因此这里不会主动设置控制器增益。
    cmd.kx[0] = pos_gain[0];
    cmd.kx[1] = pos_gain[1];
    cmd.kx[2] = pos_gain[2];

    // 把速度增益写入 PositionCommand。
    // 当前 vel_gain 默认全 0；如果下游控制器使用 cmd.kv，需要确认是否另有地方覆盖。
    cmd.kv[0] = vel_gain[0];
    cmd.kv[1] = vel_gain[1];
    cmd.kv[2] = vel_gain[2];

    // yaw 前视时间参数。
    // 若参数文件没有设置，默认 -1.0；通常建议在 launch/yaml 中给一个正值，
    // 否则 calculate_yaw 中的 t_cur + time_forward_ 会看向当前时间之前的位置。
    nh.param("traj_server/time_forward", time_forward_, -1.0);

    // yaw 历史状态初始化。
    last_yaw_ = 0.0;
    last_yaw_dot_ = 0.0;

    // 延迟 1 秒，让 ROS publisher/subscriber 有时间建立连接。
    ros::Duration(1.0).sleep();

    ROS_WARN("[Traj server]: ready.");

    // 进入 ROS 回调循环，等待 B 样条消息和定时器事件。
    ros::spin();

    return 0;
}
