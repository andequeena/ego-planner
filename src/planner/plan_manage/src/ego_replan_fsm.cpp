
#include <plan_manage/ego_replan_fsm.h>

namespace ego_planner
{

    void EGOReplanFSM::init(ros::NodeHandle &nh)
    {
        current_wp_ = 0;
        exec_state_ = FSM_EXEC_STATE::INIT;
        have_target_ = false;
        have_odom_ = false;

        /*  fsm param  */
        nh.param("fsm/flight_type", target_type_, -1);
        nh.param("fsm/thresh_replan", replan_thresh_, -1.0);
        nh.param("fsm/thresh_no_replan", no_replan_thresh_, -1.0);
        nh.param("fsm/planning_horizon", planning_horizen_, -1.0);
        nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);
        nh.param("fsm/emergency_time_", emergency_time_, 1.0);

        nh.param("fsm/waypoint_num", waypoint_num_, -1);
        for (int i = 0; i < waypoint_num_; i++)
        {
            nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
            nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
            nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
        }

        /* initialize main modules */
        visualization_.reset(new PlanningVisualization(nh));
        planner_manager_.reset(new EGOPlannerManager);
        planner_manager_->initPlanModules(nh, visualization_);

        /* callback */
        // ROS 定时器的默认行为是单线程串行回调：
        // 当前一次回调还没执行完，下一次触发时间即使到了，也会被延迟到当前执行完后再执行。
        // 它不会打断当前回调，也不会“叠加”多个线程来抢跑。
        exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this); // 100hz
        safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this); // 20hz

        odom_sub_ = nh.subscribe("/odom_world", 1, &EGOReplanFSM::odometryCallback, this);

        bspline_pub_ = nh.advertise<ego_planner::Bspline>("/planning/bspline", 10);
        data_disp_pub_ = nh.advertise<ego_planner::DataDisp>("/planning/data_display", 100);

        if (target_type_ == TARGET_TYPE::MANUAL_TARGET)
            waypoint_sub_ = nh.subscribe("/waypoint_generator/waypoints", 1, &EGOReplanFSM::waypointCallback, this);
        else if (target_type_ == TARGET_TYPE::PRESET_TARGET)
        {
            ros::Duration(1.0).sleep();
            while (ros::ok() && !have_odom_)
                ros::spinOnce();
            planGlobalTrajbyGivenWps();
        }
        else
            cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
    }

    // 预设的航标点调用这个
    void EGOReplanFSM::planGlobalTrajbyGivenWps()
    {
        std::vector<Eigen::Vector3d> wps(waypoint_num_);
        for (int i = 0; i < waypoint_num_; i++)
        {
            wps[i](0) = waypoints_[i][0];
            wps[i](1) = waypoints_[i][1];
            wps[i](2) = waypoints_[i][2];

            end_pt_ = wps.back();
        }
        // 会进行插值到inter_points中 使用线性插值
        bool success = planner_manager_->planGlobalTrajWaypoints(odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(), wps, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

        // 显示路标点
        for (size_t i = 0; i < (size_t)waypoint_num_; i++)
        {
            visualization_->displayGoalPoint(wps[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
            ros::Duration(0.001).sleep();
        }

        if (success)
        {

            /*** display ***/
            // 以 0.1s 为间隔采样全局轨迹，用于可视化显示
            constexpr double step_size_t = 0.1;
            // 计算采样点数量
            int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
            // 存储采样得到的全局路径点
            std::vector<Eigen::Vector3d> gloabl_traj(i_end);
            for (int i = 0; i < i_end; i++)
            {
                // 取出每个采样时刻对应的轨迹点
                gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
            }

            // 终点速度置零，表示当前目标已就绪
            end_vel_.setZero();
            // 标记已经接收到目标，并且有新的目标需要处理
            have_target_ = true;
            have_new_target_ = true;

            /*** FSM ***/
            // if (exec_state_ == WAIT_TARGET)
            // 触发状态机切换至生成新轨迹状态
            changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
            // else if (exec_state_ == EXEC_TRAJ)
            //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

            // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
            // 给 RViz 留出一小段时间，避免显示过快
            ros::Duration(0.001).sleep();
            // 将采样后的全局路径点发布给可视化模块显示
            visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
            ros::Duration(0.001).sleep();
        }
        else
        {
            ROS_ERROR("Unable to generate global trajectory!");
        }
    }

    // 通过ros发布的航标点调用这个
    void EGOReplanFSM::waypointCallback(const nav_msgs::PathConstPtr &msg)
    {
        if (msg->poses[0].pose.position.z < -0.1)
            return;

        cout << "Triggered!" << endl;
        trigger_ = true;
        init_pt_ = odom_pos_;

        bool success = false;
        end_pt_ << msg->poses[0].pose.position.x, msg->poses[0].pose.position.y, 1.0;
        // 开始规划
        success = planner_manager_->planGlobalTraj(odom_pos_, odom_vel_, Eigen::Vector3d::Zero(), end_pt_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

        visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

        if (success)
        {
            /*** display ***/
            constexpr double step_size_t = 0.1;
            int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
            vector<Eigen::Vector3d> gloabl_traj(i_end);
            for (int i = 0; i < i_end; i++)
            {
                gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
            }

            end_vel_.setZero();
            have_target_ = true;
            have_new_target_ = true;

            /*** FSM ***/
            if (exec_state_ == WAIT_TARGET)
                changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
            else if (exec_state_ == EXEC_TRAJ)
                changeFSMExecState(REPLAN_TRAJ, "TRIG");

            // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
            visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
        }
        else
        {
            ROS_ERROR("Unable to generate global trajectory!");
        }
    }

    void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
    {
        odom_pos_(0) = msg->pose.pose.position.x;
        odom_pos_(1) = msg->pose.pose.position.y;
        odom_pos_(2) = msg->pose.pose.position.z;

        odom_vel_(0) = msg->twist.twist.linear.x;
        odom_vel_(1) = msg->twist.twist.linear.y;
        odom_vel_(2) = msg->twist.twist.linear.z;

        // odom_acc_ = estimateAcc( msg );

        odom_orient_.w() = msg->pose.pose.orientation.w;
        odom_orient_.x() = msg->pose.pose.orientation.x;
        odom_orient_.y() = msg->pose.pose.orientation.y;
        odom_orient_.z() = msg->pose.pose.orientation.z;

        have_odom_ = true;
    }

    void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
    {

        if (new_state == exec_state_)
            continously_called_times_++;
        else
            continously_called_times_ = 1;

        static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};
        int pre_s = int(exec_state_);
        exec_state_ = new_state;
        cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
    }

    std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
    {
        return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
    }

    void EGOReplanFSM::printFSMExecState()
    {
        static string state_str[7] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP"};

        cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
    }

    void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
    {

        // static int fsm_num = 0;
        // fsm_num++;
        // if (fsm_num == 100)
        // {
        //     printFSMExecState();
        //     if (!have_odom_)
        //         cout << "no odom." << endl;
        //     if (!trigger_)
        //         cout << "wait for goal." << endl;
        //     fsm_num = 0;
        // }

        switch (exec_state_)
        {
        case INIT:
        {
            if (!have_odom_)
            {
                return;
            }
            if (!trigger_)
            {
                return;
            }
            changeFSMExecState(WAIT_TARGET, "FSM");
            break;
        }

        case WAIT_TARGET:
        {
            if (!have_target_)
                return;
            else
            {
                changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            }
            break;
        }

        case GEN_NEW_TRAJ:
        {
            // 正常来讲这个状态只进入一次
            cout << "hahaha1" << endl;

            start_pt_ = odom_pos_;
            start_vel_ = odom_vel_;
            start_acc_.setZero();

            // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
            // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
            // start_yaw_(1) = start_yaw_(2) = 0.0;

            bool flag_random_poly_init;
            if (timesOfConsecutiveStateCalls().first == 1)
                flag_random_poly_init = false;
            else
                flag_random_poly_init = true;

            bool success = callReboundReplan(true, flag_random_poly_init);
            if (success)
            {
                // 成功了执行本次轨迹
                changeFSMExecState(EXEC_TRAJ, "FSM");
                flag_escape_emergency_ = true;
            }
            else
            {
                changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            }
            break;
        }

        case REPLAN_TRAJ:
        {
            cout << "hahaha2" << endl;

            if (planFromCurrentTraj())
            {
                changeFSMExecState(EXEC_TRAJ, "FSM");
            }
            else
            {
                changeFSMExecState(REPLAN_TRAJ, "FSM");
            }

            break;
        }

        case EXEC_TRAJ:
        {
            /* determine if need to replan */
            LocalTrajData *info = &planner_manager_->local_data_;
            ros::Time time_now = ros::Time::now();
            double t_cur = (time_now - info->start_time_).toSec();
            t_cur = min(info->duration_, t_cur);

            Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

            /* && (end_pt_ - pos).norm() < 0.5 */
            if (t_cur > info->duration_ - 1e-2)
            {
                have_target_ = false;

                changeFSMExecState(WAIT_TARGET, "FSM");
                return;
            }
            else if ((end_pt_ - pos).norm() < no_replan_thresh_)
            {
                // cout << "near end" << endl;
                return;
            }
            else if ((info->start_pos_ - pos).norm() < replan_thresh_)
            {
                // cout << "near start" << endl;
                return;
            }
            else
            {
                changeFSMExecState(REPLAN_TRAJ, "FSM");
            }
            break;
        }

        case EMERGENCY_STOP:
        {

            if (flag_escape_emergency_) // Avoiding repeated calls
            {
                callEmergencyStop(odom_pos_);
            }
            else
            {
                if (odom_vel_.norm() < 0.1)
                    changeFSMExecState(GEN_NEW_TRAJ, "FSM");
            }

            flag_escape_emergency_ = false;
            break;
        }
        }

        data_disp_.header.stamp = ros::Time::now();
        data_disp_pub_.publish(data_disp_);
    }

    bool EGOReplanFSM::planFromCurrentTraj()
    {

        LocalTrajData *info = &planner_manager_->local_data_;
        ros::Time time_now = ros::Time::now();
        double t_cur = (time_now - info->start_time_).toSec();

        // cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

        start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
        start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
        start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

        bool success = callReboundReplan(false, false);
        if (!success)
        {
            // return false后再这里又运行
            success = callReboundReplan(true, false);
            // changeFSMExecState(EXEC_TRAJ, "FSM");
            if (!success)
            {
                success = callReboundReplan(true, true);
                if (!success)
                {
                    return false;
                }
            }
        }

        return true;
    }

    void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
    {
        LocalTrajData *info = &planner_manager_->local_data_;
        auto map = planner_manager_->grid_map_;

        if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
            return;

        /* ---------- check trajectory ---------- */
        constexpr double time_step = 0.01;
        double t_cur = (ros::Time::now() - info->start_time_).toSec();
        double t_2_3 = info->duration_ * 2 / 3;
        for (double t = t_cur; t < info->duration_; t += time_step)
        {
            if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
                break;

            if (map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t)))
            {
                cout << "hahaha3" << endl;

                if (planFromCurrentTraj()) // Make a chance
                {
                    changeFSMExecState(EXEC_TRAJ, "SAFETY");
                    return;
                }
                else
                {
                    if (t - t_cur < emergency_time_) // 0.8s of emergency time
                    {
                        ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
                        changeFSMExecState(EMERGENCY_STOP, "SAFETY");
                    }
                    else
                    {
                        // ROS_WARN("current traj in collision, replan.");
                        changeFSMExecState(REPLAN_TRAJ, "SAFETY");
                    }
                    return;
                }
                break;
            }
        }
    }

    // 重要接口
    bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
    {

        getLocalTarget();

        bool plan_success =
            planner_manager_->reboundReplan(start_pt_, start_vel_, start_acc_, local_target_pt_, local_target_vel_, (have_new_target_ || flag_use_poly_init), flag_randomPolyTraj);
        have_new_target_ = false;

        cout << "final_plan_success=" << plan_success << endl;

        if (plan_success)
        {

            auto info = &planner_manager_->local_data_;

            /* publish traj */
            ego_planner::Bspline bspline;
            bspline.order = 3;
            bspline.start_time = info->start_time_;
            bspline.traj_id = info->traj_id_;

            Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
            bspline.pos_pts.reserve(pos_pts.cols());
            for (int i = 0; i < pos_pts.cols(); ++i)
            {
                geometry_msgs::Point pt;
                pt.x = pos_pts(0, i);
                pt.y = pos_pts(1, i);
                pt.z = pos_pts(2, i);
                bspline.pos_pts.push_back(pt);
            }

            Eigen::VectorXd knots = info->position_traj_.getKnot();
            bspline.knots.reserve(knots.rows());
            for (int i = 0; i < knots.rows(); ++i)
            {
                bspline.knots.push_back(knots(i));
            }

            bspline_pub_.publish(bspline);

            visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
        }

        return plan_success;
    }

    bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
    {

        planner_manager_->EmergencyStop(stop_pos);

        auto info = &planner_manager_->local_data_;

        /* publish traj */
        ego_planner::Bspline bspline;
        bspline.order = 3;
        bspline.start_time = info->start_time_;
        bspline.traj_id = info->traj_id_;

        Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
        bspline.pos_pts.reserve(pos_pts.cols());
        for (int i = 0; i < pos_pts.cols(); ++i)
        {
            geometry_msgs::Point pt;
            pt.x = pos_pts(0, i);
            pt.y = pos_pts(1, i);
            pt.z = pos_pts(2, i);
            bspline.pos_pts.push_back(pt);
        }

        Eigen::VectorXd knots = info->position_traj_.getKnot();
        bspline.knots.reserve(knots.rows());
        for (int i = 0; i < knots.rows(); ++i)
        {
            bspline.knots.push_back(knots(i));
        }

        bspline_pub_.publish(bspline);

        return true;
    }

    void EGOReplanFSM::getLocalTarget()
    {
        // t 用来沿着全局轨迹的时间轴向前采样，寻找当前局部规划的目标点。
        double t;

        // 采样时间间隔：把规划视野 planning_horizen_ 按空间距离粗分成 20 份，
        // 再除以最大速度，换算成沿全局轨迹查询位置时使用的时间步长。
        double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;

        // dist_min 记录采样过程中全局轨迹点到当前起点 start_pt_ 的最小距离；
        // dist_min_t 记录该最近点对应的全局轨迹时间，用于更新 last_progress_time_。
        double dist_min = 9999, dist_min_t = 0.0;

        // 从上一次全局轨迹推进到的位置 last_progress_time_ 开始向前搜索，
        // 直到全局轨迹结束 global_duration_。
        for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
        {
            // 查询全局轨迹在时间 t 对应的位置，并计算它到当前局部规划起点的欧氏距离。
            Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
            double dist = (pos_t - start_pt_).norm();

            // 第一个采样点理论上应该接近当前进度附近。
            // 如果刚从 last_progress_time_ 开始采样就已经超过规划视野，
            // 说明 last_progress_time_ 可能落后太多或全局轨迹进度记录异常。
            if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
            {
                // todo
                ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
                ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
                ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
                ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
                ROS_ERROR("last_progress_time_ ERROR !!!!!!!!!");
                return;
            }

            // 持续记录离当前起点最近的全局轨迹采样点。
            // 后面找到局部目标点后，会把 last_progress_time_ 更新到这个最近点时间，
            // 表示全局轨迹进度已经推进到离当前无人机位置最近的位置。
            if (dist < dist_min)
            {
                dist_min = dist;
                dist_min_t = t;
            }

            // 当某个全局轨迹点距离当前起点达到或超过规划视野时，
            // 将它作为本次局部重规划的局部目标点。
            if (dist >= planning_horizen_)
            {
                local_target_pt_ = pos_t;

                // 更新全局轨迹推进时间，避免下次仍从更早的位置重复搜索。
                planner_manager_->global_data_.last_progress_time_ = dist_min_t;
                break;
            }
        }

        // 如果沿全局轨迹一直采样到末尾都没有找到超过规划视野的点，
        // 说明终点已经在当前规划视野内，直接把最终目标点作为局部目标点。
        if (t > planner_manager_->global_data_.global_duration_) // Last global point
        {
            local_target_pt_ = end_pt_;
        }

        // 判断局部目标点是否已经接近最终目标点。
        // v^2 / (2a) 是从最大速度以最大加速度刹停所需的理论制动距离。
        // 若剩余距离小于制动距离，则局部目标速度设为 0，促使轨迹在终点附近减速停止。
        if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
        {
            // local_target_vel_ = (end_pt_ - init_pt_).normalized() * planner_manager_->pp_.max_vel_ * (( end_pt_ - local_target_pt_ ).norm() / ((planner_manager_->pp_.max_vel_*planner_manager_->pp_.max_vel_)/(2*planner_manager_->pp_.max_acc_)));
            // cout << "A" << endl;
            local_target_vel_ = Eigen::Vector3d::Zero();
        }
        else
        {
            // 若离终点还比较远，则沿用全局轨迹在时间 t 处的速度作为局部目标速度，
            // 使局部轨迹尽量平滑地衔接全局轨迹方向和速度。
            local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
            // cout << "AA" << endl;
        }
    }

} // namespace ego_planner
