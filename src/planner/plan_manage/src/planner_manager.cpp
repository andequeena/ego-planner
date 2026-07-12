// #include <fstream>
#include <plan_manage/planner_manager.h>
#include <thread>

namespace ego_planner
{
    // SECTION interfaces for setup and query

    EGOPlannerManager::EGOPlannerManager() {}

    EGOPlannerManager::~EGOPlannerManager() { std::cout << "des manager" << std::endl; }

    void EGOPlannerManager::initPlanModules(ros::NodeHandle &nh, PlanningVisualization::Ptr vis)
    {
        /* read algorithm parameters */
        // nh.param<类型>("参数名称", 变量, 默认值);
        nh.param("manager/max_vel", pp_.max_vel_, -1.0);
        nh.param("manager/max_acc", pp_.max_acc_, -1.0);
        nh.param("manager/max_jerk", pp_.max_jerk_, -1.0);
        nh.param("manager/feasibility_tolerance", pp_.feasibility_tolerance_, 0.0);
        nh.param("manager/control_points_distance", pp_.ctrl_pt_dist, -1.0);
        nh.param("manager/planning_horizon", pp_.planning_horizen_, 5.0);

        local_data_.traj_id_ = 0;
        grid_map_.reset(new GridMap);
        grid_map_->initMap(nh);

        bspline_optimizer_rebound_.reset(new BsplineOptimizer);
        bspline_optimizer_rebound_->setParam(nh);
        bspline_optimizer_rebound_->setEnvironment(grid_map_);
        bspline_optimizer_rebound_->a_star_.reset(new AStar);
        bspline_optimizer_rebound_->a_star_->initGridMap(grid_map_, Eigen::Vector3i(100, 100, 100));

        visualization_ = vis;
    }

    // !SECTION

    // SECTION rebond replanning

    // the most important function
    bool EGOPlannerManager::reboundReplan(Eigen::Vector3d start_pt, Eigen::Vector3d start_vel,
                                          Eigen::Vector3d start_acc, Eigen::Vector3d local_target_pt,
                                          Eigen::Vector3d local_target_vel, bool flag_polyInit, bool flag_randomPolyTraj)
    {

        static int count = 0;
        // 被调用次数
        std::cout << endl
                  << "[rebo replan]: -------------------------------------" << count++ << std::endl;
        cout.precision(3);
        // 输出起点终点位置速度
        cout << "start: " << start_pt.transpose() << ", " << start_vel.transpose() << "\ngoal:" << local_target_pt.transpose() << ", " << local_target_vel.transpose()
             << endl;

        if ((start_pt - local_target_pt).norm() < 0.2)
        {
            cout << "Close to goal" << endl;
            continous_failures_count_++;
            return false;
        }

        ros::Time t_start = ros::Time::now();
        ros::Duration t_init, t_opt, t_refine;

        cout << "yxc see reboundReplan step1" << endl;
        /*** STEP 1: INIT ***/
        // ts 是后面从初始轨迹上采样点的时间间隔，也会作为 B 样条初始控制点的时间尺度参考。
        // 理想情况下，如果相邻控制点空间距离约为 pp_.ctrl_pt_dist，且无人机以最大速度 pp_.max_vel_ 匀速运动，
        // 那么对应的时间间隔大约是 pp_.ctrl_pt_dist / pp_.max_vel_。
        //
        // 但直接使用 pp_.ctrl_pt_dist / pp_.max_vel_ 会让初始轨迹“太紧”：
        // 相同空间距离分配的时间太短，优化后的 B 样条很容易超过速度或加速度约束。
        // 所以这里乘经验放大系数，让时间分配更宽松：
        //   1.2：正常距离下略微放松时间间隔，降低超速度/超加速度风险。
        //   5  ：当起点和局部目标几乎重合时，距离太短，若仍用很小的时间间隔会导致数值和导数约束很敏感；
        //        因此显著放大 ts，让近距离场景下的初始轨迹更平缓。
        // 0.1m 是判断“起点和局部目标是否几乎重合”的距离阈值。
        double ts = (start_pt - local_target_pt).norm() > 0.1 ? pp_.ctrl_pt_dist / pp_.max_vel_ * 1.2 : pp_.ctrl_pt_dist / pp_.max_vel_ * 5; // pp_.ctrl_pt_dist / pp_.max_vel_ is too tense, and will surely exceed the acc/vel limits
        vector<Eigen::Vector3d> point_set, start_end_derivatives;
        static bool flag_first_call = true, flag_force_polynomial = false;
        bool flag_regenerate = false;
        do
        {
            point_set.clear();
            start_end_derivatives.clear();
            flag_regenerate = false;

            if (flag_first_call || flag_polyInit || flag_force_polynomial /*|| ( start_pt - local_target_pt ).norm() < 1.0*/) // Initial path generated from a min-snap traj by order.
            {
                // 首次调用
                flag_first_call = false;
                flag_force_polynomial = false;

                PolynomialTraj gl_traj;

                double dist = (start_pt - local_target_pt).norm();

                // 根据起点到局部目标点的直线距离 dist，估计一段多项式初始轨迹的总时间 time。
                // 这里使用的是典型的“一维梯形/三角速度规划”的时间估计思想：
                //
                // 1. 如果距离足够长，可以先以 max_acc 加速到 max_vel，再匀速飞行，最后以 max_acc 减速。
                //    从 0 加速到 max_vel 的距离是 0.5 * v^2 / a，减速同理也是 0.5 * v^2 / a，
                //    两段合计距离为 v^2 / a，也就是 pow(pp_.max_vel_, 2) / pp_.max_acc_。
                //
                // 2. 如果 v^2 / a > dist，说明距离太短，来不及加速到最大速度，速度曲线应是“三角形”：
                //    加速一段再立刻减速。这里用 sqrt(dist / max_acc) 作为时间尺度估计。
                //    注意该写法是工程上的保守/经验估计，用于生成初始多项式，不是严格完整的双边界动力学求解。
                //
                // 3. 如果 v^2 / a <= dist，说明可以达到最大速度，速度曲线近似为“梯形”：
                //    总时间 = 匀速段时间 + 加速时间 + 减速时间
                //           = (dist - v^2 / a) / v + 2 * v / a。
                //
                // 这个 time 只用于构造初始轨迹 gl_traj，后续还会经过 B 样条优化和可行性检查。
                double time = pow(pp_.max_vel_, 2) / pp_.max_acc_ > dist ? sqrt(dist / pp_.max_acc_) : (dist - pow(pp_.max_vel_, 2) / pp_.max_acc_) / pp_.max_vel_ + 2 * pp_.max_vel_ / pp_.max_acc_;

                if (!flag_randomPolyTraj)
                {
                    // 直线路径
                    gl_traj = PolynomialTraj::one_segment_traj_gen(start_pt, start_vel, start_acc, local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), time);
                }
                else
                {
                    Eigen::Vector3d horizen_dir = ((start_pt - local_target_pt).cross(Eigen::Vector3d(0, 0, 1))).normalized();
                    Eigen::Vector3d vertical_dir = ((start_pt - local_target_pt).cross(horizen_dir)).normalized();
                    Eigen::Vector3d random_inserted_pt = (start_pt + local_target_pt) / 2 +
                                                         (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * horizen_dir * 0.8 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989) +
                                                         (((double)rand()) / RAND_MAX - 0.5) * (start_pt - local_target_pt).norm() * vertical_dir * 0.4 * (-0.978 / (continous_failures_count_ + 0.989) + 0.989);
                    Eigen::MatrixXd pos(3, 3);
                    pos.col(0) = start_pt;
                    pos.col(1) = random_inserted_pt;
                    pos.col(2) = local_target_pt;
                    Eigen::VectorXd t(2);
                    t(0) = t(1) = time / 2;
                    gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, local_target_vel, start_acc, Eigen::Vector3d::Zero(), t);
                }

                // 从全局轨迹（gl_traj）中采样出一组间距均匀的路径点 point_set，
                // 保证点间距离合适（不过远也不过密），并同时获取轨迹起止的速度和加速度约束。
                double t;
                bool flag_too_far;
                // 先把 ts 放大 1.5，进入 do-while 后第一步会 ts /= 1.5，
                // 这样第一次循环实际使用的是上面计算出的原始 ts。
                // 如果发现相邻采样点太远，循环继续执行，每次都把 ts 再缩小 1.5 倍，
                // 通过更密的时间采样来减小相邻路径点距离。
                ts *= 1.5; // ts will be divided by 1.5 in the next
                do
                {
                    // 缩小采样时间间隔，使轨迹采样点更密。
                    // 时间间隔越小，point_set 中点越多，相邻点空间距离越容易满足 ctrl_pt_dist 附近的要求。
                    ts /= 1.5;
                    point_set.clear();
                    flag_too_far = false;
                    Eigen::Vector3d last_pt = gl_traj.evaluate(0); // gl_traj.evaluate(0)：得到全局轨迹在 t=0 时刻的起点。
                    for (t = 0; t < time; t += ts)
                    {
                        Eigen::Vector3d pt = gl_traj.evaluate(t);
                        // 如果相邻采样点距离超过期望控制点间距的 1.5 倍，
                        // 说明当前 ts 太大、采样太稀，后面会重新缩小 ts 再采样。
                        if ((last_pt - pt).norm() > pp_.ctrl_pt_dist * 1.5)
                        {
                            flag_too_far = true;
                            break;
                        }
                        last_pt = pt;
                        point_set.push_back(pt);
                    }
                    // 至少保留 7 个点，是为了给后续 B 样条初始化提供足够控制点。
                    // 点太少时，样条自由度不足，难以同时满足起终点导数、避障和动力学约束。
                } while (flag_too_far || point_set.size() < 7); // To make sure the initial path has enough points.
                t -= ts;
                // 起始点和终止点速度加速度
                start_end_derivatives.push_back(gl_traj.evaluateVel(0));
                start_end_derivatives.push_back(local_target_vel);
                start_end_derivatives.push_back(gl_traj.evaluateAcc(0));
                start_end_derivatives.push_back(gl_traj.evaluateAcc(t));
            }
            else // Initial path generated from previous trajectory.
            {

                double t;

                // t_cur 是当前时刻在上一条局部 B 样条轨迹中的相对时间。
                // local_data_.start_time_ 是上一条轨迹开始执行的 ROS 时间；
                // 当前时间减去 start_time_，就得到无人机理论上已经沿上一条轨迹执行到哪里。
                double t_cur = (ros::Time::now() - local_data_.start_time_).toSec();

                // pseudo_arc_length 保存“近似弧长”累积表：
                //   pseudo_arc_length[i] 表示从当前时刻 t_cur 开始，
                //   沿 segment_point[0] 到 segment_point[i] 走过的大致路径长度。
                // 这里说“伪/近似”，是因为它不是对曲线精确积分，而是用相邻采样点之间的直线距离累加。
                vector<double> pseudo_arc_length;

                // segment_point 保存从旧轨迹截取出来的未来轨迹点，
                // 后面如果旧轨迹不够到达 local_target_pt，还会继续拼接一段多项式轨迹点。
                vector<Eigen::Vector3d> segment_point;

                // 起点的累计弧长为 0。
                pseudo_arc_length.push_back(0.0);

                // 从当前执行时间 t_cur 开始，沿上一条局部轨迹继续向后采样到轨迹结束。
                // 这相当于把旧轨迹还没执行完的部分拿出来，作为新初始轨迹的前半段，
                // 这样重规划时不会突然丢弃当前正在执行的轨迹，轨迹连续性更好。
                for (t = t_cur; t < local_data_.duration_ + 1e-3; t += ts)
                {
                    // evaluateDeBoorT(t) 用 De Boor 算法计算 B 样条在时间 t 对应的位置。
                    segment_point.push_back(local_data_.position_traj_.evaluateDeBoorT(t));
                    if (t > t_cur)
                    {
                        // 累加相邻采样点之间的距离，构造近似弧长坐标。
                        // 这个弧长表后面用于按空间距离等间隔重新采样，而不是按时间等间隔采样。
                        pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
                    }
                }
                // for 循环结束时 t 已经多加了一次 ts，这里退回到最后一个有效采样时间。
                t -= ts;

                // 旧轨迹最后一个采样点到新局部目标点之间，可能还差一段距离。
                // poly_time 用“距离 / 最大速度 * 2”估计补接多项式的持续时间：
                //   距离 / max_vel 是理论最短匀速时间；
                //   乘 2 是经验放宽，让补接段更平滑，降低速度/加速度约束压力。
                double poly_time = (local_data_.position_traj_.evaluateDeBoorT(t) - local_target_pt).norm() / pp_.max_vel_ * 2;

                // 如果补接段时间大于当前采样间隔 ts，说明这段距离有必要单独生成多项式并采样；
                // 如果 poly_time <= ts，说明旧轨迹末端已经离新局部目标很近，可以不用额外插入中间点。
                if (poly_time > ts)
                {
                    // 从旧轨迹最后一个有效采样点出发，生成一段到 local_target_pt 的单段多项式轨迹。
                    // 起点位置/速度/加速度都继承旧轨迹末端状态，终点速度使用 local_target_vel，
                    // 终点加速度设为 0，用于平滑地把旧轨迹余段接到新的局部目标。
                    PolynomialTraj gl_traj = PolynomialTraj::one_segment_traj_gen(local_data_.position_traj_.evaluateDeBoorT(t),
                                                                                  local_data_.velocity_traj_.evaluateDeBoorT(t),
                                                                                  local_data_.acceleration_traj_.evaluateDeBoorT(t),
                                                                                  local_target_pt, local_target_vel, Eigen::Vector3d::Zero(), poly_time);

                    // 对补接的多项式轨迹按 ts 进行时间采样，并继续追加到 segment_point 和 pseudo_arc_length。
                    // 注意这里从 t = ts 开始，而不是 0，因为 t = 0 的点就是旧轨迹末端，已经在 segment_point 中。
                    for (t = ts; t < poly_time; t += ts)
                    {
                        if (!pseudo_arc_length.empty())
                        {
                            segment_point.push_back(gl_traj.evaluate(t));
                            // 继续累加补接段的近似弧长，使整条“旧轨迹余段 + 新目标补接段”
                            // 都在同一个弧长坐标系下。
                            pseudo_arc_length.push_back((segment_point.back() - segment_point[segment_point.size() - 2]).norm() + pseudo_arc_length.back());
                        }
                        else
                        {
                            // 正常情况下 pseudo_arc_length 至少有起点 0.0。
                            // 如果为空，说明前面的初始化逻辑异常，直接返回失败，避免后续越界或生成错误轨迹。
                            ROS_ERROR("pseudo_arc_length is empty, return!");
                            continous_failures_count_++;
                            return false;
                        }
                    }
                }

                // sample_length 是沿近似弧长表前进的采样位置。
                // 后面会按照 cps_dist 的空间间隔，从 segment_point 中线性插值得到新的 point_set。
                double sample_length = 0;

                // cps_dist 是期望的控制点/路径点空间间隔。
                // 先乘 1.5，进入 do-while 后马上除以 1.5，
                // 这样第一次循环使用的就是 pp_.ctrl_pt_dist。
                // 如果点数不足 7，循环会继续把 cps_dist 缩小 1.5 倍，让采样更密、点数更多。
                double cps_dist = pp_.ctrl_pt_dist * 1.5; // cps_dist will be divided by 1.5 in the next

                // id 指向当前 sample_length 所落入的弧长区间：
                // [pseudo_arc_length[id], pseudo_arc_length[id + 1])
                size_t id = 0;
                do
                {
                    // 缩小空间采样间隔。第一次循环恢复到 ctrl_pt_dist，
                    // 后续若点数不足，则持续缩小，使近距离目标也能生成足够点数。
                    cps_dist /= 1.5;
                    point_set.clear();
                    sample_length = 0;
                    id = 0;
                    while ((id <= pseudo_arc_length.size() - 2) && sample_length <= pseudo_arc_length.back())
                    {
                        // 找到 sample_length 所在的相邻采样点区间，然后在这两个点之间线性插值。
                        // 这样得到的 point_set 近似按空间距离均匀分布，比直接按时间采样更稳定。
                        if (sample_length >= pseudo_arc_length[id] && sample_length < pseudo_arc_length[id + 1])
                        {
                            point_set.push_back((sample_length - pseudo_arc_length[id]) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id + 1] +
                                                (pseudo_arc_length[id + 1] - sample_length) / (pseudo_arc_length[id + 1] - pseudo_arc_length[id]) * segment_point[id]);
                            sample_length += cps_dist;
                        }
                        else
                        {
                            // sample_length 还不在当前区间内，继续向后找下一个弧长区间。
                            id++;
                        }
                    }

                    // 确保最终局部目标点一定被加入 point_set，
                    // 避免由于采样步长没有刚好落到终点而丢失目标约束。
                    point_set.push_back(local_target_pt);
                } while (point_set.size() < 7); // If the start point is very close to end point, this will help

                // 设置 B 样条参数化时使用的起终点导数约束：
                // 起点速度/加速度继承旧轨迹在当前时刻 t_cur 的状态，保证与正在执行的轨迹连续；
                // 终点速度使用 local_target_vel，终点加速度设为 0，作为局部目标处的平滑边界条件。
                start_end_derivatives.push_back(local_data_.velocity_traj_.evaluateDeBoorT(t_cur));
                start_end_derivatives.push_back(local_target_vel);
                start_end_derivatives.push_back(local_data_.acceleration_traj_.evaluateDeBoorT(t_cur));
                start_end_derivatives.push_back(Eigen::Vector3d::Zero());

                // 如果“从历史轨迹生成”的 point_set 异常长（说明路径不合理），则设 flag_force_polynomial=true 并 flag_regenerate=true，
                // 循环会再跑一次，下一次会进入 if（因为 flag_force_polynomial 为 true），改用多项式生成初始轨迹。
                if (point_set.size() > pp_.planning_horizen_ / pp_.ctrl_pt_dist * 3) // The initial path is unnormally too long!
                {
                    // 强制使用多项式
                    flag_force_polynomial = true;
                    // 这里不设置为true就运行一次
                    flag_regenerate = true;
                }
                cout << "yxc 123 yxc 123" << endl;
            }
        } while (flag_regenerate);

        Eigen::MatrixXd ctrl_pts;
        // 将离散点集 + 起终速度/加速度约束，生成均匀三次 B 样条控制点。
        UniformBspline::parameterizeToBspline(ts, point_set, start_end_derivatives, ctrl_pts);

        vector<vector<Eigen::Vector3d>> a_star_pathes;
        a_star_pathes = bspline_optimizer_rebound_->initControlPoints(ctrl_pts, true);

        t_init = ros::Time::now() - t_start;

        static int vis_id = 0;
        // 初始路径点在 RViz 中显示得稍大一些，便于和地图、优化后轨迹区分。
        visualization_->displayInitPathList(point_set, 0.35, 0);
        visualization_->displayAStarList(a_star_pathes, vis_id);

        t_start = ros::Time::now();

        cout << "yxc see step2" << endl;
        /*** STEP 2: OPTIMIZE ***/
        bool flag_step_1_success = bspline_optimizer_rebound_->BsplineOptimizeTrajRebound(ctrl_pts, ts);
        cout << "first_optimize_step_success=" << flag_step_1_success << endl;
        if (!flag_step_1_success)
        {
            // visualization_->displayOptimalList( ctrl_pts, vis_id );
            continous_failures_count_++;
            return false;
        }
        // visualization_->displayOptimalList( ctrl_pts, vis_id );

        t_opt = ros::Time::now() - t_start;
        t_start = ros::Time::now();

        cout << "yxc see step3" << endl;
        /*** STEP 3: REFINE(RE-ALLOCATE TIME) IF NECESSARY ***/
        UniformBspline pos = UniformBspline(ctrl_pts, 3, ts);
        pos.setPhysicalLimits(pp_.max_vel_, pp_.max_acc_, pp_.feasibility_tolerance_);

        double ratio;
        bool flag_step_2_success = true;
        // checkFeasibility 会遍历轨迹，检查每个采样点的速度/加速度是否超过限制
        if (!pos.checkFeasibility(ratio, false))
        {
            cout << "Need to reallocate time." << endl;

            Eigen::MatrixXd optimal_control_points;
            flag_step_2_success = refineTrajAlgo(pos, start_end_derivatives, ratio, ts, optimal_control_points);
            if (flag_step_2_success)
                pos = UniformBspline(optimal_control_points, 3, ts);
        }

        if (!flag_step_2_success)
        {
            printf("\033[34mThis refined trajectory hits obstacles. It doesn't matter if appeares occasionally. But if continously appearing, Increase parameter \"lambda_fitness\".\n\033[0m");
            continous_failures_count_++;
            return false;
        }

        t_refine = ros::Time::now() - t_start;

        // save planned results
        // 保留这段轨迹
        updateTrajInfo(pos, ros::Time::now());

        cout << "total time:\033[42m" << (t_init + t_opt + t_refine).toSec() << "\033[0m,optimize:" << (t_init + t_opt).toSec() << ",refine:" << t_refine.toSec() << endl;

        // success. YoY
        continous_failures_count_ = 0;
        return true;
    }

    bool EGOPlannerManager::EmergencyStop(Eigen::Vector3d stop_pos)
    {
        Eigen::MatrixXd control_points(3, 6);
        for (int i = 0; i < 6; i++)
        {
            control_points.col(i) = stop_pos;
        }

        updateTrajInfo(UniformBspline(control_points, 3, 1.0), ros::Time::now());

        return true;
    }

    bool EGOPlannerManager::planGlobalTrajWaypoints(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                    const std::vector<Eigen::Vector3d> &waypoints, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
    {

        // generate global reference trajectory

        vector<Eigen::Vector3d> points;
        points.push_back(start_pos);

        for (size_t wp_i = 0; wp_i < waypoints.size(); wp_i++)
        {
            points.push_back(waypoints[wp_i]);
        }

        double total_len = 0;
        total_len += (start_pos - waypoints[0]).norm();
        for (size_t i = 0; i < waypoints.size() - 1; i++)
        {
            // .norm()：Eigen中向量的欧几里得范数，即长度（√(x²+y²+z²)）。
            total_len += (waypoints[i + 1] - waypoints[i]).norm();
        }

        // insert intermediate points if too far
        vector<Eigen::Vector3d> inter_points;
        double dist_thresh = max(total_len / 8, 4.0);

        for (size_t i = 0; i < points.size() - 1; ++i)
        {
            inter_points.push_back(points.at(i));
            double dist = (points.at(i + 1) - points.at(i)).norm();

            if (dist > dist_thresh)
            {
                // floor 向下取整
                int id_num = floor(dist / dist_thresh) + 1;

                for (int j = 1; j < id_num; ++j)
                {
                    Eigen::Vector3d inter_pt =
                        points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
                    inter_points.push_back(inter_pt);
                }
            }
        }

        inter_points.push_back(points.back());

        cout << "yxc test inter_points: " << endl;
        for (int i = 0; i < inter_points.size(); i++)
        {
            cout << inter_points[i].transpose() << endl;
        }

        // write position matrix
        int pt_num = inter_points.size();
        Eigen::MatrixXd pos(3, pt_num);
        for (int i = 0; i < pt_num; ++i)
            pos.col(i) = inter_points[i];

        Eigen::Vector3d zero(0, 0, 0);
        Eigen::VectorXd time(pt_num - 1);
        for (int i = 0; i < pt_num - 1; ++i)
        {
            time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
        }

        time(0) *= 2.0;
        time(time.rows() - 1) *= 2.0;

        PolynomialTraj gl_traj;
        if (pos.cols() >= 3)
            gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
        else if (pos.cols() == 2)
            gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, pos.col(1), end_vel, end_acc, time(0));
        else
            return false;

        auto time_now = ros::Time::now();
        global_data_.setGlobalTraj(gl_traj, time_now);

        return true;
    }

    bool EGOPlannerManager::planGlobalTraj(const Eigen::Vector3d &start_pos, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                           const Eigen::Vector3d &end_pos, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc)
    {

        // generate global reference trajectory

        vector<Eigen::Vector3d> points;
        points.push_back(start_pos);
        points.push_back(end_pos);

        // insert intermediate points if too far
        vector<Eigen::Vector3d> inter_points;
        const double dist_thresh = 4.0;

        for (size_t i = 0; i < points.size() - 1; ++i)
        {
            inter_points.push_back(points.at(i));
            double dist = (points.at(i + 1) - points.at(i)).norm();

            if (dist > dist_thresh)
            {
                int id_num = floor(dist / dist_thresh) + 1;

                for (int j = 1; j < id_num; ++j)
                {
                    Eigen::Vector3d inter_pt =
                        points.at(i) * (1.0 - double(j) / id_num) + points.at(i + 1) * double(j) / id_num;
                    inter_points.push_back(inter_pt);
                }
            }
        }

        inter_points.push_back(points.back());

        // write position matrix
        int pt_num = inter_points.size();
        Eigen::MatrixXd pos(3, pt_num);
        for (int i = 0; i < pt_num; ++i)
            pos.col(i) = inter_points[i];

        Eigen::Vector3d zero(0, 0, 0);
        Eigen::VectorXd time(pt_num - 1);
        for (int i = 0; i < pt_num - 1; ++i)
        {
            time(i) = (pos.col(i + 1) - pos.col(i)).norm() / (pp_.max_vel_);
        }

        time(0) *= 2.0;
        time(time.rows() - 1) *= 2.0;

        PolynomialTraj gl_traj;
        if (pos.cols() >= 3)
            gl_traj = PolynomialTraj::minSnapTraj(pos, start_vel, end_vel, start_acc, end_acc, time);
        else if (pos.cols() == 2)
            gl_traj = PolynomialTraj::one_segment_traj_gen(start_pos, start_vel, start_acc, end_pos, end_vel, end_acc, time(0));
        else
            return false;

        auto time_now = ros::Time::now();
        global_data_.setGlobalTraj(gl_traj, time_now);

        return true;
    }

    bool EGOPlannerManager::refineTrajAlgo(UniformBspline &traj, vector<Eigen::Vector3d> &start_end_derivative, double ratio, double &ts, Eigen::MatrixXd &optimal_control_points)
    {
        double t_inc;

        Eigen::MatrixXd ctrl_pts; // = traj.getControlPoint()

        // std::cout << "ratio: " << ratio << std::endl;
        reparamBspline(traj, start_end_derivative, ratio, ctrl_pts, ts, t_inc);

        traj = UniformBspline(ctrl_pts, 3, ts);

        double t_step = traj.getTimeSum() / (ctrl_pts.cols() - 3);
        bspline_optimizer_rebound_->ref_pts_.clear();
        for (double t = 0; t < traj.getTimeSum() + 1e-4; t += t_step)
            bspline_optimizer_rebound_->ref_pts_.push_back(traj.evaluateDeBoorT(t));

        bool success = bspline_optimizer_rebound_->BsplineOptimizeTrajRefine(ctrl_pts, ts, optimal_control_points);

        return success;
    }

    void EGOPlannerManager::updateTrajInfo(const UniformBspline &position_traj, const ros::Time time_now)
    {
        local_data_.start_time_ = time_now;
        local_data_.position_traj_ = position_traj;
        local_data_.velocity_traj_ = local_data_.position_traj_.getDerivative();
        local_data_.acceleration_traj_ = local_data_.velocity_traj_.getDerivative();
        local_data_.start_pos_ = local_data_.position_traj_.evaluateDeBoorT(0.0);
        local_data_.duration_ = local_data_.position_traj_.getTimeSum();
        local_data_.traj_id_ += 1;
    }

    void EGOPlannerManager::reparamBspline(UniformBspline &bspline, vector<Eigen::Vector3d> &start_end_derivative, double ratio,
                                           Eigen::MatrixXd &ctrl_pts, double &dt, double &time_inc)
    {
        double time_origin = bspline.getTimeSum();
        int seg_num = bspline.getControlPoint().cols() - 3;
        // double length = bspline.getLength(0.1);
        // int seg_num = ceil(length / pp_.ctrl_pt_dist);

        bspline.lengthenTime(ratio);
        double duration = bspline.getTimeSum();
        dt = duration / double(seg_num);
        time_inc = duration - time_origin;

        vector<Eigen::Vector3d> point_set;
        for (double time = 0.0; time <= duration + 1e-4; time += dt)
        {
            point_set.push_back(bspline.evaluateDeBoorT(time));
        }
        UniformBspline::parameterizeToBspline(dt, point_set, start_end_derivative, ctrl_pts);
    }

} // namespace ego_planner
