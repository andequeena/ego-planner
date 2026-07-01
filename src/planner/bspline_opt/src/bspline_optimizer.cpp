#include "bspline_opt/bspline_optimizer.h"
#include "bspline_opt/gradient_descent_optimizer.h"
// using namespace std;

namespace ego_planner
{
    void BsplineOptimizer::setParam(ros::NodeHandle &nh)
    {
        // 从 ROS 参数服务器读取各项代价权重和动力学/安全距离约束。
        // 默认值设为 -1.0，方便在配置缺失时暴露异常参数。
        nh.param("optimization/lambda_smooth", lambda1_, -1.0);
        nh.param("optimization/lambda_collision", lambda2_, -1.0);
        nh.param("optimization/lambda_feasibility", lambda3_, -1.0);
        nh.param("optimization/lambda_fitness", lambda4_, -1.0);

        nh.param("optimization/dist0", dist0_, -1.0);
        nh.param("optimization/max_vel", max_vel_, -1.0);
        nh.param("optimization/max_acc", max_acc_, -1.0);

        nh.param("optimization/order", order_, 3);
    }

    void BsplineOptimizer::setEnvironment(const GridMap::Ptr &env)
    {
        // 保存栅格地图/SDF 环境指针，后续碰撞检查和距离代价都会使用它。
        this->grid_map_ = env;
    }

    void BsplineOptimizer::setControlPoints(const Eigen::MatrixXd &points)
    {
        // 设置当前待优化的 B 样条控制点矩阵，每一列表示一个三维控制点。
        cps_.points = points;
    }

    // 设置 B 样条 knot 的均匀时间间隔，影响速度和加速度约束的尺度。
    void BsplineOptimizer::setBsplineInterval(const double &ts) { bspline_interval_ = ts; }

    /* This function is very similar to check_collision_and_rebound().
     * It was written separately, just because I did it once and it has been running stably since March 2020.
     * But I will merge then someday.*/
    std::vector<std::vector<Eigen::Vector3d>> BsplineOptimizer::initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init /*= true*/)
    {

        if (flag_first_init)
        {
            // 首次初始化时建立 ControlPoints 容器，并把安全距离写入 clearance。
            cps_.clearance = dist0_;
            cps_.resize(init_points.cols());
            cps_.points = init_points;
        }

        /*** Segment the initial trajectory according to obstacles ***/
        constexpr int ENOUGH_INTERVAL = 2;
        // 轨迹点间平均距离
        // 除以地图分辨率并 /2，保证轨迹采样足够密集
        // 这里的 step_size 不是物理长度，而是相邻两个控制点线性插值参数 a 的递减步长：
        //   采样点 p(a) = a * P(i-1) + (1-a) * P(i)，a 从 1 递减到 0。
        // 分母 ((首点-末点).norm() / (点数-1)) 用整段首尾直线距离估计平均控制点间距。
        // grid_map_->getResolution() / 平均间距 表示“一个栅格分辨率大约占相邻控制点间隔的多少比例”。
        // 最后再除以 2，相当于每半个地图栅格采样一次，降低细小障碍被跨过去的概率。
        // 注意：init_points.rightCols(1) 取最后一列，Eigen 会广播成 3x1 向量参与差值计算。
        double step_size = grid_map_->getResolution() / ((init_points.col(0) - init_points.rightCols(1)).norm() / (init_points.cols() - 1)) / 2;
        int in_id, out_id; // 障碍段起点索引 障碍段终点索引
        // segment_ids 保存所有检测到的碰撞段，每个 pair 的 first/second 是进入和离开障碍附近的控制点索引。
        vector<std::pair<int, int>> segment_ids;
        // same_occ_state_times 统计连续处于同一种占据状态的采样次数。
        // 初始设为 ENOUGH_INTERVAL + 1，表示一开始就满足“稳定空闲/稳定占据”的判定条件。
        int same_occ_state_times = ENOUGH_INTERVAL + 1;
        bool occ, last_occ = false;
        // flag_got_start：已经确认进入障碍；flag_got_end_maybe：刚从障碍出来但还未稳定确认；
        // flag_got_end：已经确认离开障碍。三者配合构成一个简单状态机。
        bool flag_got_start = false, flag_got_end = false, flag_got_end_maybe = false;
        // i_end 控制只检查靠近当前时刻的前 2/3 有效控制点：
        //   init_points.cols() - 2 * order_ 是去掉首尾固定控制点后的可优化区间长度；
        //   /3 表示尾部约 1/3 暂不检查，因为它离当前机器人较远，后续重规划会继续处理；
        //   init_points.cols() - order_ 是最后一个需要避开尾端固定段的上界；
        //   再减去可优化区间的 1/3，就得到“前 2/3”的截止索引。
        // 这样可以减少远端轨迹对当前优化的干扰，也节省碰撞检测和 A* 搜索时间。
        int i_end = (int)init_points.cols() - order_ - ((int)init_points.cols() - 2 * order_) / 3; // only check closed 2/3 points.
        // 从第 order_ 个控制点开始，只检查靠近机器人当前位置的前 2/3 段轨迹。
        // 内层在相邻控制点之间插值采样，识别“进入障碍”和“离开障碍”的索引。
        for (int i = order_; i <= i_end; ++i)
        {
            for (double a = 1.0; a >= 0.0; a -= step_size)
            {
                // 检测该点是否占据障碍（膨胀后的障碍）
                // a=1 时采样 P(i-1)，a=0 时采样 P(i)，中间值为两控制点连线上的点。
                // 使用膨胀占据而不是原始占据，是为了把机器人半径/安全裕度计入碰撞判断。
                occ = grid_map_->getInflateOccupancy(a * init_points.col(i - 1) + (1 - a) * init_points.col(i));
                // cout << setprecision(5);
                // cout << (a * init_points.col(i-1) + (1-a) * init_points.col(i)).transpose() << " occ1=" << occ << endl;

                // 遇到障碍物起始点
                if (occ && !last_occ)
                {
                    // 只有当前一个状态已经连续稳定了 ENOUGH_INTERVAL 次以上，才认为这是有效的状态切换；
                    // i == order_ 是起始边界特判，避免第一段本来就在障碍内时漏掉入口。
                    if (same_occ_state_times > ENOUGH_INTERVAL || i == order_)
                    {
                        in_id = i - 1;
                        flag_got_start = true;
                    }
                    // 状态刚发生变化，连续计数清零，等待后续采样验证。
                    same_occ_state_times = 0;
                    flag_got_end_maybe = false; // terminate in advance
                }
                else if (!occ && last_occ)
                {
                    // 从占据变为空闲，暂记为障碍段出口；之后还要等待若干连续空闲采样确认。
                    out_id = i;
                    flag_got_end_maybe = true;
                    same_occ_state_times = 0;
                }
                else
                {
                    ++same_occ_state_times;
                }

                if (flag_got_end_maybe && (same_occ_state_times > ENOUGH_INTERVAL || (i == (int)init_points.cols() - order_)))
                {
                    // 连续空闲足够长，认为已经真正离开障碍段，避免由单个采样噪声导致误判。
                    flag_got_end_maybe = false;
                    flag_got_end = true;
                }

                last_occ = occ;

                if (flag_got_start && flag_got_end)
                {
                    flag_got_start = false;
                    flag_got_end = false;
                    // 存的是满足条件的段 
                    // 每个 pair 表示一段与障碍相交的控制点索引范围 [in_id, out_id]。
                    segment_ids.push_back(std::pair<int, int>(in_id, out_id));
                }
            }
        }

        /*** a star search ***/
        // yxc test
        cout << "yxc see using Asearch!" << endl;
        vector<vector<Eigen::Vector3d>> a_star_pathes;
        for (size_t i = 0; i < segment_ids.size(); ++i)
        {
            // cout << "in=" << in.transpose() << " out=" << out.transpose() << endl;
            // 对每个碰撞段，以进入障碍前后的空闲控制点作为起终点，搜索一条绕障参考路径。
            // A* 路径不是最终轨迹，只作为“应该往哪边绕开障碍”的几何引导线。
            // 后续会拿这条路径和控制点局部法线求交，构造 base_point/direction 避障约束。
            Eigen::Vector3d in(init_points.col(segment_ids[i].first)), out(init_points.col(segment_ids[i].second));
            if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.1, in, out))
            {
                a_star_pathes.push_back(a_star_->getPath());
            }
            else
            {
                ROS_ERROR("a star error, force return!");
                return a_star_pathes;
            }
        }

        /*** calculate bounds ***/
        int id_low_bound, id_up_bound;
        vector<std::pair<int, int>> bounds(segment_ids.size());
        for (size_t i = 0; i < segment_ids.size(); i++)
        {

            // bounds 用来限制每个碰撞段可向左右扩展的控制点范围，
            // 防止相邻障碍段的反弹方向互相覆盖。
            if (i == 0) // first segment
            {
                id_low_bound = order_;
                // 若有下一段 → 当前段终点和下一段起点中间；否则轨迹末端。
                if (segment_ids.size() > 1)
                {
                    // 上界取当前障碍段出口和下一障碍段入口的中点偏左，
                    // 防止当前段扩展过头后侵入下一段的反弹范围。
                    id_up_bound = (int)(((segment_ids[0].second + segment_ids[1].first) - 1.0f) / 2); // id_up_bound : -1.0f fix()
                }
                else
                {
                    // 没有下一段时，最多扩展到末端固定控制点之前。
                    id_up_bound = init_points.cols() - order_ - 1;
                }
            }
            else if (i == segment_ids.size() - 1) // last segment, i != 0 here
            {
                // 下界取上一障碍段出口和当前障碍段入口的中点偏右，
                // 这样相邻两个障碍段的扩展区间不会重叠。
                id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
                id_up_bound = init_points.cols() - order_ - 1;
            }
            else
            {
                // 中间障碍段两侧都用相邻段的中点切分，形成当前段独占的可扩展范围。
                id_low_bound = (int)(((segment_ids[i].first + segment_ids[i - 1].second) + 1.0f) / 2); // id_low_bound : +1.0f ceil()
                id_up_bound = (int)(((segment_ids[i].second + segment_ids[i + 1].first) - 1.0f) / 2);  // id_up_bound : -1.0f fix()
            }

            bounds[i] = std::pair<int, int>(id_low_bound, id_up_bound);
        }

        // cout << "+++++++++" << endl;
        // for ( int j=0; j<bounds.size(); ++j )
        // {
        //   cout << bounds[j].first << "  " << bounds[j].second << endl;
        // }

        /*** Adjust segment length ***/
        vector<std::pair<int, int>> final_segment_ids(segment_ids.size());
        constexpr double MINIMUM_PERCENT = 0.0; // Each segment is guaranteed to have sufficient points to generate sufficient thrust
        // minimum_points 按总控制点数给每个碰撞段设置最少控制点数量。
        // 当前 MINIMUM_PERCENT 为 0，实际不会强制扩展；保留这套逻辑便于需要更强避障推力时调整。
        int minimum_points = round(init_points.cols() * MINIMUM_PERCENT), num_points;
        for (size_t i = 0; i < segment_ids.size(); i++)
        {
            /*** Adjust segment length ***/
            // 若碰撞段过短，则在 bounds 允许范围内向两侧扩展，
            // 让后续距离梯度作用在足够多的控制点上。
            num_points = segment_ids[i].second - segment_ids[i].first + 1;
            // cout << "i = " << i << " first = " << segment_ids[i].first << " second = " << segment_ids[i].second << endl;
            if (num_points < minimum_points)
            {
                // 缺少的点数平均分配到左右两侧；+1 后再 /2 等价于向上取整，
                // 保证扩展后的点数尽量达到 minimum_points。
                double add_points_each_side = (int)(((minimum_points - num_points) + 1.0f) / 2);

                // 左扩展不能越过 bounds[i].first。
                final_segment_ids[i].first = segment_ids[i].first - add_points_each_side >= bounds[i].first ? segment_ids[i].first - add_points_each_side : bounds[i].first;

                // 右扩展不能越过 bounds[i].second。
                final_segment_ids[i].second = segment_ids[i].second + add_points_each_side <= bounds[i].second ? segment_ids[i].second + add_points_each_side : bounds[i].second;
            }
            else
            {
                final_segment_ids[i].first = segment_ids[i].first;
                final_segment_ids[i].second = segment_ids[i].second;
            }

            // cout << "final:" << "i = " << i << " first = " << final_segment_ids[i].first << " second = " << final_segment_ids[i].second << endl;
        }

        /*** Assign data to each segment ***/
        for (size_t i = 0; i < segment_ids.size(); i++)
        {
            // step 1
            // 清空当前段临时标记，后面用来记录哪些控制点已经成功生成了反弹方向。
            for (int j = final_segment_ids[i].first; j <= final_segment_ids[i].second; ++j)
                cps_.flag_temp[j] = false;

            // step 2
            // 在原控制点法线方向和 A* 路径之间寻找交点。
            // 交点到控制点的方向就是将该控制点推出障碍的参考方向。
            int got_intersection_id = -1;
            for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
            {
                // ctrl_pts_law 由 P(j-1)->P(j+1) 给出，近似当前控制点处轨迹切向。
                // 代码后面用 dot(ctrl_pts_law) 判断 A* 点落在该“法向截面”的哪一侧。
                Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
                // 从 A* 路径中点开始找交点，通常比从头开始更容易靠近障碍段中心。
                int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
                // val = (A*点 - 控制点) dot ctrl_pts_law。
                // val 的正负表示 A* 点在过控制点、法向为 ctrl_pts_law 的平面两侧。
                double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
                // 沿 A* 路径前后移动，寻找点积符号变化的位置，即跨过控制点法线平面的位置。
                while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
                {
                    last_Astar_id = Astar_id;

                    // val >= 0 时向路径前半段搜索，否则向后半段搜索；
                    // 目标是找到相邻两个 A* 点刚好跨过控制点截面的地方。
                    if (val >= 0)
                        --Astar_id;
                    else
                        ++Astar_id;

                    val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

                    if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
                    {
                        // 用相邻两个 A* 路径点线性插值，求出与控制点法线平面的交点。
                        // 设交点 X = A + t(A-B)，其中 A 是当前 A* 点，B 是上一个 A* 点；
                        // 要求 (X - Pj) dot ctrl_pts_law = 0，解出的 t 就是下面括号中的比例项。
                        intersection_point =
                            a_star_pathes[i][Astar_id] +
                            ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                             (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                            );

                        // cout << "i=" << i << " j=" << j << " Astar_id=" << Astar_id << " last_Astar_id=" << last_Astar_id << " intersection_point = " << intersection_point.transpose() << endl;

                        got_intersection_id = j;
                        break;
                    }
                }

                if (got_intersection_id >= 0)
                {
                    cps_.flag_temp[j] = true;
                    double length = (intersection_point - cps_.points.col(j)).norm();
                    if (length > 1e-5)
                    {
                        // 从交点向当前控制点回扫，找到最靠近障碍边界的 base_point。
                        // base_point + direction 一起定义了距离代价中的有向安全约束。
                        // a 表示从控制点朝交点方向走了多少距离；步长取地图分辨率，
                        // 因此 base_point 的定位精度约为一个栅格。
                        for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
                        {
                            // (a/length)*intersection + (1-a/length)*control 是线段上的插值点。
                            // 从交点向控制点扫描，第一次遇到占据或接近控制点时停止。
                            occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

                            if (occ || a < grid_map_->getResolution())
                            {
                                // 若当前采样点已经在障碍内，就退回一个分辨率，尽量把 base_point 放到障碍外侧边界。
                                if (occ)
                                    a += grid_map_->getResolution();
                                cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                                // direction 从当前控制点指向 A* 绕障交点，归一化后作为避障代价的推出方向。
                                cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                                break;
                            }
                        }
                    }
                }
            }

            /* Corner case: the segment length is too short. Here the control points may outside the A* path, leading to opposite gradient direction. So I have to take special care of it */
            if (segment_ids[i].second - segment_ids[i].first == 1)
            {
                // 只有两个控制点的碰撞段无法在内部点上求交，
                // 因此使用两端中点来估计反弹方向，避免梯度方向反过来推入障碍。
                Eigen::Vector3d ctrl_pts_law(cps_.points.col(segment_ids[i].second) - cps_.points.col(segment_ids[i].first)), intersection_point;
                Eigen::Vector3d middle_point = (cps_.points.col(segment_ids[i].second) + cps_.points.col(segment_ids[i].first)) / 2;
                int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
                double val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law), last_val = val;
                while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
                {
                    last_Astar_id = Astar_id;

                    if (val >= 0)
                        --Astar_id;
                    else
                        ++Astar_id;

                    val = (a_star_pathes[i][Astar_id] - middle_point).dot(ctrl_pts_law);

                    if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
                    {
                        intersection_point =
                            a_star_pathes[i][Astar_id] +
                            ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                             (ctrl_pts_law.dot(middle_point - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                            );

                        if ((intersection_point - middle_point).norm() > 0.01) // 1cm.
                        {
                            cps_.flag_temp[segment_ids[i].first] = true;
                            cps_.base_point[segment_ids[i].first].push_back(cps_.points.col(segment_ids[i].first));
                            cps_.direction[segment_ids[i].first].push_back((intersection_point - middle_point).normalized());

                            got_intersection_id = segment_ids[i].first;
                        }
                        break;
                    }
                }
            }

            // step 3
            if (got_intersection_id >= 0)
            {
                // 若某个控制点已经成功得到反弹方向，则向右传播给同段内尚未命中的控制点。
                for (int j = got_intersection_id + 1; j <= final_segment_ids[i].second; ++j)
                    if (!cps_.flag_temp[j])
                    {
                        cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
                        cps_.direction[j].push_back(cps_.direction[j - 1].back());
                    }

                // 同理向左传播，使整个扩展段都拥有可用的 base_point/direction。
                for (int j = got_intersection_id - 1; j >= final_segment_ids[i].first; --j)
                    if (!cps_.flag_temp[j])
                    {
                        cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
                        cps_.direction[j].push_back(cps_.direction[j + 1].back());
                    }
            }
            else
            {
                // Just ignore, it does not matter ^_^.
                // ROS_ERROR("Failed to generate direction! segment_id=%d", i);
            }
        }

        return a_star_pathes;
    }

    int BsplineOptimizer::earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls)
    {
        BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);
        // cout << "k=" << k << endl;
        // cout << "opt->flag_continue_to_optimize_=" << opt->flag_continue_to_optimize_ << endl;
        // L-BFGS 每轮回调这里；当检测到新碰撞需要 rebound 或出现错误时提前停止求解。
        return (opt->force_stop_type_ == STOP_FOR_ERROR || opt->force_stop_type_ == STOP_FOR_REBOUND);
    }

    double BsplineOptimizer::costFunctionRebound(void *func_data, const double *x, double *grad, const int n)
    {
        // L-BFGS 的 C 风格回调接口：把 func_data 转回优化器对象后计算 rebound 代价。
        BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

        double cost;
        opt->combineCostRebound(x, grad, cost, n);

        opt->iter_num_ += 1;
        return cost;
    }

    double BsplineOptimizer::costFunctionRefine(void *func_data, const double *x, double *grad, const int n)
    {
        // refine 阶段使用同样的回调形式，但代价项换成平滑、贴合参考线和动力学可行性。
        BsplineOptimizer *opt = reinterpret_cast<BsplineOptimizer *>(func_data);

        double cost;
        opt->combineCostRefine(x, grad, cost, n);

        opt->iter_num_ += 1;
        return cost;
    }

    void BsplineOptimizer::calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost,
                                                   Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost)
    {
        cost = 0.0;
        int end_idx = q.cols() - order_;
        // demarcation 内使用三次惩罚，超过 demarcation 后转为二次形式，
        // 这样既保持近边界处平滑，又避免深度侵入时梯度过弱。
        double demarcation = cps_.clearance;
        // 当 dist_err >= demarcation 时使用 a*x^2+b*x+c。
        // 这里 a,b,c 被选成让二次段在 x=demarcation 处与 x^3 的函数值和一阶导连续：
        //   x^3 在 d 处值 d^3、导数 3d^2；
        //   3d*x^2 - 3d^2*x + d^3 在 d 处同样为 d^3、导数 3d^2。
        // 这样从轻微侵入到严重侵入的代价曲线不会出现突变。
        double a = 3 * demarcation, b = -3 * pow(demarcation, 2), c = pow(demarcation, 3);

        force_stop_type_ = DONT_STOP;
        if (iter_num > 3 && smoothness_cost / (cps_.size - 2 * order_) < 0.1) // 0.1 is an experimental value that indicates the trajectory is smooth enough.
        {
            // 当轨迹已经比较平滑后，再做一次碰撞检查；若发现新碰撞，则中断本轮优化并重新生成反弹约束。
            check_collision_and_rebound();
        }

        /*** calculate distance cost and gradient ***/
        for (auto i = order_; i < end_idx; ++i)
        {
            for (size_t j = 0; j < cps_.direction[i].size(); ++j)
            {
                // dist 是控制点沿反弹方向到 base_point 的有符号距离；
                // 小于 clearance 时说明安全距离不足，需要施加推离障碍的梯度。
                double dist = (cps_.points.col(i) - cps_.base_point[i][j]).dot(cps_.direction[i][j]);
                double dist_err = cps_.clearance - dist;
                Eigen::Vector3d dist_grad = cps_.direction[i][j];

                if (dist_err < 0)
                {
                    // dist 已经大于 clearance，说明该控制点在该方向上满足安全距离，不产生代价。
                    /* do nothing */
                }
                else if (dist_err < demarcation)
                {
                    // 轻微不足安全距离时使用 dist_err^3。
                    // 对控制点 q 的梯度：d(dist_err)/dq = -direction，
                    // 因此 d(dist_err^3)/dq = -3*dist_err^2*direction。
                    cost += pow(dist_err, 3);
                    gradient.col(i) += -3.0 * dist_err * dist_err * dist_grad;
                }
                else
                {
                    // 严重不足安全距离时改用连续拼接的二次代价，梯度随侵入深度线性增加。
                    cost += a * dist_err * dist_err + b * dist_err + c;
                    gradient.col(i) += -(2.0 * a * dist_err + b) * dist_grad;
                }
            }
        }
    }

    void BsplineOptimizer::calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient)
    {

        cost = 0.0;

        int end_idx = q.cols() - order_;

        // def: f = |x*v|^2/a^2 + |x×v|^2/b^2
        // fitness 代价让曲线尽量贴近 refine 阶段给定的参考点序列；
        // 沿参考方向和横向偏差分别用不同权重 a2/b2 约束。
        double a2 = 25, b2 = 1;
        // a2 比 b2 大，表示沿参考线切向的误差惩罚较弱、横向偏离惩罚较强；
        // 这样 refine 会优先让轨迹靠近参考通道，而不是强制精确匹配参考点的进度。
        for (auto i = order_ - 1; i < end_idx + 1; ++i)
        {
            // 三次均匀 B 样条在 knot 处的位置可由相邻三个控制点 [1,4,1]/6 表示。
            Eigen::Vector3d x = (q.col(i - 1) + 4 * q.col(i) + q.col(i + 1)) / 6.0 - ref_pts_[i - 1];
            // v 是参考点局部切向，使用 ref_pts_[i] - ref_pts_[i-2] 做中心差分估计。
            Eigen::Vector3d v = (ref_pts_[i] - ref_pts_[i - 2]).normalized();

            // xdotv 是沿参考切线方向的偏差；xcrossv 的模长是到参考切线的横向偏差。
            double xdotv = x.dot(v);
            Eigen::Vector3d xcrossv = x.cross(v);

            double f = pow((xdotv), 2) / a2 + pow(xcrossv.norm(), 2) / b2;
            cost += f;

            Eigen::Matrix3d m;
            m << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
            // m 是 v 的反对称矩阵形式，使 m * xcrossv 等价于 v × (x × v) 的梯度项。
            // df_dx 是对曲线点位置的梯度，再按 [1,4,1]/6 分配回三个控制点。
            Eigen::Vector3d df_dx = 2 * xdotv / a2 * v + 2 / b2 * m * xcrossv;

            gradient.col(i - 1) += df_dx / 6;
            gradient.col(i) += 4 * df_dx / 6;
            gradient.col(i + 1) += df_dx / 6;
        }
    }

    void BsplineOptimizer::calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                              Eigen::MatrixXd &gradient, bool falg_use_jerk /* = true*/)
    {

        cost = 0.0;

        if (falg_use_jerk)
        {
            Eigen::Vector3d jerk, temp_j;

            for (int i = 0; i < q.cols() - 3; i++)
            {
                /* evaluate jerk */
                // 三阶有限差分近似 jerk，惩罚 jerk 可让轨迹更平滑、控制输入变化更小。
                // jerk = P(i+3)-3P(i+2)+3P(i+1)-P(i)，对应离散三阶导数的分子。
                jerk = q.col(i + 3) - 3 * q.col(i + 2) + 3 * q.col(i + 1) - q.col(i);
                cost += jerk.squaredNorm();
                // cost = jerk dot jerk，所以 d(cost)/d(jerk)=2*jerk。
                temp_j = 2.0 * jerk;
                /* jerk gradient */
                // 对四个相关控制点分别累加 jerk^2 的解析梯度。
                // jerk 对 P(i),P(i+1),P(i+2),P(i+3) 的系数分别是 -1,3,-3,1。
                gradient.col(i + 0) += -temp_j;
                gradient.col(i + 1) += 3.0 * temp_j;
                gradient.col(i + 2) += -3.0 * temp_j;
                gradient.col(i + 3) += temp_j;
            }
        }
        else
        {
            Eigen::Vector3d acc, temp_acc;

            for (int i = 0; i < q.cols() - 2; i++)
            {
                /* evaluate acc */
                // 二阶有限差分近似加速度，作为不用 jerk 时的平滑项。
                // acc = P(i+2)-2P(i+1)+P(i)，只衡量局部弯折程度，平滑性弱于 jerk 项。
                acc = q.col(i + 2) - 2 * q.col(i + 1) + q.col(i);
                cost += acc.squaredNorm();
                temp_acc = 2.0 * acc;
                /* acc gradient */
                // acc 对三个控制点的系数分别是 1,-2,1。
                gradient.col(i + 0) += temp_acc;
                gradient.col(i + 1) += -2.0 * temp_acc;
                gradient.col(i + 2) += temp_acc;
            }
        }
    }

    void BsplineOptimizer::calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                               Eigen::MatrixXd &gradient)
    {

        // #define SECOND_DERIVATIVE_CONTINOUS

#ifdef SECOND_DERIVATIVE_CONTINOUS

        cost = 0.0;
        // 连续版本在接近约束边界时使用三次惩罚，超出 demarcation 后切换为二次惩罚。
        double demarcation = 1.0; // 1m/s, 1m/s/s
        double ar = 3 * demarcation, br = -3 * pow(demarcation, 2), cr = pow(demarcation, 3);
        double al = ar, bl = -br, cl = cr;

        /* abbreviation */
        // ts_inv2/ts_inv3 用于把控制点差分转换为真实时间尺度下的速度、加速度。
        double ts, ts_inv2, ts_inv3;
        ts = bspline_interval_;
        // ts_inv2 = 1/ts^2，用于二阶差分到加速度的尺度换算。
        ts_inv2 = 1 / ts / ts;
        // ts_inv3 主要用于调整速度惩罚量级，使速度项和加速度项在数值优化中更接近。
        ts_inv3 = 1 / ts / ts / ts;

        /* velocity feasibility */
        // 速度约束：相邻控制点差分 / ts，逐维限制在 [-max_vel_, max_vel_]。
        for (int i = 0; i < q.cols() - 1; i++)
        {
            Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

            for (int j = 0; j < 3; j++)
            {
                if (vi(j) > max_vel_ + demarcation)
                {
                    // 正向速度严重超限：diff 是超过 max_vel_ 的量。
                    // 使用二次拼接段，避免超限很大时三次项过强导致数值不稳定。
                    double diff = vi(j) - max_vel_;
                    cost += (ar * diff * diff + br * diff + cr) * ts_inv3; // multiply ts_inv3 to make vel and acc has similar magnitude

                    // vi = (P(i+1)-P(i))/ts，所以梯度对 P(i) 为 -1/ts，对 P(i+1) 为 +1/ts。
                    double grad = (2.0 * ar * diff + br) / ts * ts_inv3;
                    gradient(j, i + 0) += -grad;
                    gradient(j, i + 1) += grad;
                }
                else if (vi(j) > max_vel_)
                {
                    // 正向速度轻微超限：使用 diff^3，使刚越界时梯度从 0 平滑增长。
                    double diff = vi(j) - max_vel_;
                    cost += pow(diff, 3) * ts_inv3;
                    ;

                    double grad = 3 * diff * diff / ts * ts_inv3;
                    ;
                    gradient(j, i + 0) += -grad;
                    gradient(j, i + 1) += grad;
                }
                else if (vi(j) < -(max_vel_ + demarcation))
                {
                    // 负向速度严重超限，diff = vi + max_vel_ 为负数。
                    // al/bl/cl 让负侧惩罚也保持连续。
                    double diff = vi(j) + max_vel_;
                    cost += (al * diff * diff + bl * diff + cl) * ts_inv3;

                    double grad = (2.0 * al * diff + bl) / ts * ts_inv3;
                    gradient(j, i + 0) += -grad;
                    gradient(j, i + 1) += grad;
                }
                else if (vi(j) < -max_vel_)
                {
                    // 负向速度轻微超限，-diff^3 为正代价，因为 diff 为负。
                    double diff = vi(j) + max_vel_;
                    cost += -pow(diff, 3) * ts_inv3;

                    double grad = -3 * diff * diff / ts * ts_inv3;
                    gradient(j, i + 0) += -grad;
                    gradient(j, i + 1) += grad;
                }
                else
                {
                    /* nothing happened */
                }
            }
        }

        /* acceleration feasibility */
        // 加速度约束：二阶控制点差分 / ts^2，逐维限制在 [-max_acc_, max_acc_]。
        for (int i = 0; i < q.cols() - 2; i++)
        {
            // 二阶差分除以 ts^2 得到每一维的近似加速度。
            Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

            for (int j = 0; j < 3; j++)
            {
                if (ai(j) > max_acc_ + demarcation)
                {
                    // 正向加速度严重超限，同样使用二次拼接段。
                    double diff = ai(j) - max_acc_;
                    cost += ar * diff * diff + br * diff + cr;

                    // ai 对 P(i),P(i+1),P(i+2) 的系数为 1,-2,1，再乘 ts_inv2。
                    double grad = (2.0 * ar * diff + br) * ts_inv2;
                    gradient(j, i + 0) += grad;
                    gradient(j, i + 1) += -2 * grad;
                    gradient(j, i + 2) += grad;
                }
                else if (ai(j) > max_acc_)
                {
                    // 正向加速度轻微超限，三次惩罚保证边界处梯度平滑。
                    double diff = ai(j) - max_acc_;
                    cost += pow(diff, 3);

                    double grad = 3 * diff * diff * ts_inv2;
                    gradient(j, i + 0) += grad;
                    gradient(j, i + 1) += -2 * grad;
                    gradient(j, i + 2) += grad;
                }
                else if (ai(j) < -(max_acc_ + demarcation))
                {
                    // 负向加速度严重超限。
                    double diff = ai(j) + max_acc_;
                    cost += al * diff * diff + bl * diff + cl;

                    double grad = (2.0 * al * diff + bl) * ts_inv2;
                    gradient(j, i + 0) += grad;
                    gradient(j, i + 1) += -2 * grad;
                    gradient(j, i + 2) += grad;
                }
                else if (ai(j) < -max_acc_)
                {
                    // 负向加速度轻微超限，diff 为负，因此代价写成 -diff^3。
                    double diff = ai(j) + max_acc_;
                    cost += -pow(diff, 3);

                    double grad = -3 * diff * diff * ts_inv2;
                    gradient(j, i + 0) += grad;
                    gradient(j, i + 1) += -2 * grad;
                    gradient(j, i + 2) += grad;
                }
                else
                {
                    /* nothing happened */
                }
            }
        }

#else

        cost = 0.0;
        /* abbreviation */
        // 默认分支使用较简单的二次惩罚：只在超出速度/加速度限制时产生代价。
        double ts, /*vm2, am2, */ ts_inv2;
        // vm2 = max_vel_ * max_vel_;
        // am2 = max_acc_ * max_acc_;

        ts = bspline_interval_;
        // 默认分支只需要 ts_inv2 来缩放速度二次惩罚和加速度梯度量级。
        ts_inv2 = 1 / ts / ts;

        /* velocity feasibility */
        // 对每一段控制点差分形成速度，若任一维超限则累加二次代价和对应梯度。
        for (int i = 0; i < q.cols() - 1; i++)
        {
            Eigen::Vector3d vi = (q.col(i + 1) - q.col(i)) / ts;

            // cout << "temp_v * vi=" ;
            for (int j = 0; j < 3; j++)
            {
                if (vi(j) > max_vel_)
                {
                    // cout << "fuck VEL" << endl;
                    // cout << vi(j) << endl;
                    // 速度超过正上限时，惩罚 (vi-max_vel)^2。
                    // 额外乘 ts_inv2 是经验尺度项，让速度约束在优化中有足够权重。
                    cost += pow(vi(j) - max_vel_, 2) * ts_inv2; // multiply ts_inv3 to make vel and acc has similar magnitude

                    gradient(j, i + 0) += -2 * (vi(j) - max_vel_) / ts * ts_inv2;
                    gradient(j, i + 1) += 2 * (vi(j) - max_vel_) / ts * ts_inv2;
                }
                else if (vi(j) < -max_vel_)
                {
                    // 速度超过负上限时，惩罚 (vi+max_vel)^2。
                    cost += pow(vi(j) + max_vel_, 2) * ts_inv2;

                    gradient(j, i + 0) += -2 * (vi(j) + max_vel_) / ts * ts_inv2;
                    gradient(j, i + 1) += 2 * (vi(j) + max_vel_) / ts * ts_inv2;
                }
                else
                {
                    /* code */
                }
            }
        }

        /* acceleration feasibility */
        // 对连续三个控制点形成加速度，若任一维超限则把梯度分配到 i,i+1,i+2。
        for (int i = 0; i < q.cols() - 2; i++)
        {
            // ai 是三点二阶差分，对应 B 样条控制多边形上的近似加速度。
            Eigen::Vector3d ai = (q.col(i + 2) - 2 * q.col(i + 1) + q.col(i)) * ts_inv2;

            // cout << "temp_a * ai=" ;
            for (int j = 0; j < 3; j++)
            {
                if (ai(j) > max_acc_)
                {
                    // cout << "fuck ACC" << endl;
                    // cout << ai(j) << endl;
                    // 加速度正向超限时，代价对三个控制点的梯度按 1,-2,1 分配。
                    cost += pow(ai(j) - max_acc_, 2);

                    gradient(j, i + 0) += 2 * (ai(j) - max_acc_) * ts_inv2;
                    gradient(j, i + 1) += -4 * (ai(j) - max_acc_) * ts_inv2;
                    gradient(j, i + 2) += 2 * (ai(j) - max_acc_) * ts_inv2;
                }
                else if (ai(j) < -max_acc_)
                {
                    // 加速度负向超限。
                    cost += pow(ai(j) + max_acc_, 2);

                    gradient(j, i + 0) += 2 * (ai(j) + max_acc_) * ts_inv2;
                    gradient(j, i + 1) += -4 * (ai(j) + max_acc_) * ts_inv2;
                    gradient(j, i + 2) += 2 * (ai(j) + max_acc_) * ts_inv2;
                }
                else
                {
                    /* code */
                }
            }
            // cout << endl;
        }

#endif
    }

    bool BsplineOptimizer::check_collision_and_rebound(void)
    {

        int end_idx = cps_.size - order_;

        /*** Check and segment the initial trajectory according to obstacles ***/
        // 在优化过程中重新检查当前控制点是否撞到膨胀障碍，
        // 若出现新的有效碰撞段，就重新搜索 A* 并追加 rebound 约束。
        int in_id, out_id;
        vector<std::pair<int, int>> segment_ids;
        bool flag_new_obs_valid = false;
        // 与 initControlPoints 类似，只检查靠近当前时刻的前 2/3 有效控制点。
        // end_idx = size - order_ 是避开末端固定控制点后的尾部边界。
        int i_end = end_idx - (end_idx - order_) / 3;
        for (int i = order_ - 1; i <= i_end; ++i)
        {

            bool occ = grid_map_->getInflateOccupancy(cps_.points.col(i));

            /*** check if the new collision will be valid ***/
            if (occ)
            {
                // 如果该点虽然在占据区内，但相对已有 base_point/direction 已经位于“外侧”，
                // 则认为不是新的有效碰撞，避免重复添加方向相同的约束。
                for (size_t k = 0; k < cps_.direction[i].size(); ++k)
                {
                    cout.precision(2);
                    // 当前控制点到已有 base_point 的投影距离如果小于一个分辨率，
                    // 说明它仍处在已有约束附近，暂不认为是“新障碍”。
                    if ((cps_.points.col(i) - cps_.base_point[i][k]).dot(cps_.direction[i][k]) < 1 * grid_map_->getResolution()) // current point is outside all the collision_points.
                    {
                        occ = false; // Not really takes effect, just for better hunman understanding.
                        break;
                    }
                }
            }

            if (occ)
            {
                flag_new_obs_valid = true;

                // 向前找到碰撞段入口处最近的空闲控制点。
                int j;
                for (j = i - 1; j >= 0; --j)
                {
                    occ = grid_map_->getInflateOccupancy(cps_.points.col(j));
                    if (!occ)
                    {
                        in_id = j;
                        break;
                    }
                }
                if (j < 0) // fail to get the obs free point
                {
                    ROS_ERROR("ERROR! the drone is in obstacle. This should not happen.");
                    in_id = 0;
                }

                // 向后找到碰撞段出口处最近的空闲控制点。
                for (j = i + 1; j < cps_.size; ++j)
                {
                    occ = grid_map_->getInflateOccupancy(cps_.points.col(j));

                    if (!occ)
                    {
                        out_id = j;
                        break;
                    }
                }
                if (j >= cps_.size) // fail to get the obs free point
                {
                    ROS_WARN("WARN! terminal point of the current trajectory is in obstacle, skip this planning.");

                    force_stop_type_ = STOP_FOR_ERROR;
                    return false;
                }

                i = j + 1;

                // 记录新碰撞段，用于后续 A* 搜索绕障路径。
                segment_ids.push_back(std::pair<int, int>(in_id, out_id));
            }
        }

        if (flag_new_obs_valid)
        {
            // 对每个新碰撞段重新搜索一条局部绕障路径。
            vector<vector<Eigen::Vector3d>> a_star_pathes;
            for (size_t i = 0; i < segment_ids.size(); ++i)
            {
                /*** a star search ***/
                Eigen::Vector3d in(cps_.points.col(segment_ids[i].first)), out(cps_.points.col(segment_ids[i].second));
                if (a_star_->AstarSearch(/*(in-out).norm()/10+0.05*/ 0.1, in, out))
                {
                    a_star_pathes.push_back(a_star_->getPath());
                }
                else
                {
                    ROS_ERROR("a star error");
                    segment_ids.erase(segment_ids.begin() + i);
                    i--;
                }
            }

            /*** Assign parameters to each segment ***/
            for (size_t i = 0; i < segment_ids.size(); ++i)
            {
                // step 1
                // 重置该段临时标记，避免沿用上一次碰撞检查留下的状态。
                for (int j = segment_ids[i].first; j <= segment_ids[i].second; ++j)
                    cps_.flag_temp[j] = false;

                // step 2
                // 与 initControlPoints 中相同：通过 A* 路径和控制点局部法线求交，生成反弹方向。
                int got_intersection_id = -1;
                for (int j = segment_ids[i].first + 1; j < segment_ids[i].second; ++j)
                {
                    // 重新检测阶段的反弹方向生成逻辑与初始化阶段一致：
                    // 用局部控制点切向定义截面，在 A* 绕障路径上找到穿过该截面的点。
                    Eigen::Vector3d ctrl_pts_law(cps_.points.col(j + 1) - cps_.points.col(j - 1)), intersection_point;
                    int Astar_id = a_star_pathes[i].size() / 2, last_Astar_id; // Let "Astar_id = id_of_the_most_far_away_Astar_point" will be better, but it needs more computation
                    double val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law), last_val = val;
                    while (Astar_id >= 0 && Astar_id < (int)a_star_pathes[i].size())
                    {
                        last_Astar_id = Astar_id;

                        if (val >= 0)
                            --Astar_id;
                        else
                            ++Astar_id;

                        val = (a_star_pathes[i][Astar_id] - cps_.points.col(j)).dot(ctrl_pts_law);

                        // cout << val << endl;

                        if (val * last_val <= 0 && (abs(val) > 0 || abs(last_val) > 0)) // val = last_val = 0.0 is not allowed
                        {
                            // 线性插值得到 A* 路径与控制点截面的交点，作为后续方向约束的几何参考。
                            intersection_point =
                                a_star_pathes[i][Astar_id] +
                                ((a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id]) *
                                 (ctrl_pts_law.dot(cps_.points.col(j) - a_star_pathes[i][Astar_id]) / ctrl_pts_law.dot(a_star_pathes[i][Astar_id] - a_star_pathes[i][last_Astar_id])) // = t
                                );

                            got_intersection_id = j;
                            break;
                        }
                    }

                    if (got_intersection_id >= 0)
                    {
                        cps_.flag_temp[j] = true;
                        double length = (intersection_point - cps_.points.col(j)).norm();
                        if (length > 1e-5)
                        {
                            // 沿交点到控制点的连线扫描，寻找障碍边界附近的 base_point。
                            for (double a = length; a >= 0.0; a -= grid_map_->getResolution())
                            {
                                bool occ = grid_map_->getInflateOccupancy((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));

                                if (occ || a < grid_map_->getResolution())
                                {
                                    if (occ)
                                        a += grid_map_->getResolution();
                                    cps_.base_point[j].push_back((a / length) * intersection_point + (1 - a / length) * cps_.points.col(j));
                                    cps_.direction[j].push_back((intersection_point - cps_.points.col(j)).normalized());
                                    break;
                                }
                            }
                        }
                        else
                        {
                            got_intersection_id = -1;
                        }
                    }
                }

                // step 3
                if (got_intersection_id >= 0)
                {
                    // 将找到的反弹约束向右传播到同一碰撞段的其他控制点。
                    for (int j = got_intersection_id + 1; j <= segment_ids[i].second; ++j)
                        if (!cps_.flag_temp[j])
                        {
                            cps_.base_point[j].push_back(cps_.base_point[j - 1].back());
                            cps_.direction[j].push_back(cps_.direction[j - 1].back());
                        }

                    // 再向左传播，保证该碰撞段内所有相关控制点都有梯度方向。
                    for (int j = got_intersection_id - 1; j >= segment_ids[i].first; --j)
                        if (!cps_.flag_temp[j])
                        {
                            cps_.base_point[j].push_back(cps_.base_point[j + 1].back());
                            cps_.direction[j].push_back(cps_.direction[j + 1].back());
                        }
                }
                else
                    ROS_WARN("Failed to generate direction. It doesn't matter.");
            }

            // 告诉 L-BFGS 早停：当前优化结果需要带着新增 rebound 约束重新开始。
            force_stop_type_ = STOP_FOR_REBOUND;
            return true;
        }

        return false;
    }

    bool BsplineOptimizer::BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts)
    {
        // rebound 阶段：在已有 cps_ 和碰撞约束基础上优化控制点，使轨迹远离障碍。
        setBsplineInterval(ts);

        bool flag_success = rebound_optimize();

        optimal_points = cps_.points;

        return flag_success;
    }

    bool BsplineOptimizer::BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points)
    {

        // refine 阶段：以 rebound 后的轨迹为初值，进一步平滑并贴合参考轨迹。
        setControlPoints(init_points);
        setBsplineInterval(ts);

        bool flag_success = refine_optimize();

        optimal_points = cps_.points;

        return flag_success;
    }

    bool BsplineOptimizer::rebound_optimize()
    {
        iter_num_ = 0; // 优化迭代次数计数器
        // 固定首尾 order_ 个控制点，只优化中间控制点，保证边界状态不被破坏。
        int start_id = order_;
        int end_id = this->cps_.size - order_;
        // 每个控制点 3 个变量(x,y,z)，因此变量总数为 3 * 可优化控制点数量。
        variable_num_ = 3 * (end_id - start_id);
        double final_cost; // 优化后得到的总代价

        ros::Time t0 = ros::Time::now(), t1, t2;
        int restart_nums = 0, rebound_times = 0;
        ;
        bool flag_force_return, flag_occ, success;
        // rebound 阶段会根据重启次数放大避障权重，原始 lambda2_ 保持不变。
        new_lambda2_ = lambda2_;
        constexpr int MAX_RESART_NUMS_SET = 3;
        do
        {
            /* ---------- prepare ---------- */
            min_cost_ = std::numeric_limits<double>::max();
            iter_num_ = 0;
            flag_force_return = false;
            flag_occ = false;
            success = false;

            double q[variable_num_];
            // Eigen 矩阵按列主序存储，控制点每列 3 个 double；这里拷贝中间控制点给 L-BFGS。
            // data() + 3 * start_id 跳过前 start_id 个三维控制点，即跳过固定起点段。
            memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

            lbfgs::lbfgs_parameter_t lbfgs_params;
            lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
            // mem_size 是 L-BFGS 保留的历史校正对数量；max_iterations/g_epsilon 控制终止条件。
            lbfgs_params.mem_size = 16;
            lbfgs_params.max_iterations = 200;
            lbfgs_params.g_epsilon = 0.01;

            /* ---------- optimize ---------- */
            t1 = ros::Time::now();
            // costFunctionRebound 计算代价和梯度，earlyExit 用于在发现新碰撞时中断本轮求解。
            int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRebound, NULL, BsplineOptimizer::earlyExit, this, &lbfgs_params);
            t2 = ros::Time::now();
            double time_ms = (t2 - t1).toSec() * 1000;
            double total_time_ms = (t2 - t0).toSec() * 1000;

            /* ---------- success temporary, check collision again ---------- */
            if (result == lbfgs::LBFGS_CONVERGENCE ||
                result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
                result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
                result == lbfgs::LBFGS_STOP)
            {
                // ROS_WARN("Solver error in planning!, return = %s", lbfgs::lbfgs_strerror(result));
                flag_force_return = false;

                // 将优化后的控制点生成 Bspline。
                UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
                double tm, tmp;
                traj.getTimeSpan(tm, tmp);
                // 按地图分辨率决定采样步长，保证轨迹检查不会跨过栅格障碍。
                // (tmp-tm) 是轨迹总时间；分母约为“轨迹首尾直线长度 / 地图分辨率”，
                // 因此 t_step 对应的空间间隔约为一个栅格。
                double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution());
                for (double t = tm; t < tmp * 2 / 3; t += t_step) // Only check the closest 2/3 partition of the whole trajectory.
                {
                    flag_occ = grid_map_->getInflateOccupancy(traj.evaluateDeBoorT(t));
                    if (flag_occ)
                    {
                        // cout << "hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

                        if (t <= bspline_interval_) // First 3 control points in obstacles!
                        {
                            // 如果很靠近起点就碰撞，说明起始局部控制点本身不安全；
                            // 这类情况靠继续优化中段通常无法修复，直接返回失败。
                            cout << cps_.points.col(1).transpose() << "\n"
                                 << cps_.points.col(2).transpose() << "\n"
                                 << cps_.points.col(3).transpose() << "\n"
                                 << cps_.points.col(4).transpose() << endl;
                            ROS_WARN("First 3 control points in obstacles! return false, t=%f", t);
                            return false;
                        }

                        break;
                    }
                }

                if (!flag_occ)
                {
                    printf("\033[32miter(+1)=%d,time(ms)=%5.3f,total_t(ms)=%5.3f,cost=%5.3f\n\033[0m", iter_num_, time_ms, total_time_ms, final_cost);
                    success = true;
                }
                else // restart
                {
                    // 若优化后仍碰撞，重新生成控制点反弹方向，并加大避障权重继续尝试。
                    restart_nums++;
                    initControlPoints(cps_.points, false);
                    // 每次重启把避障权重翻倍，使优化器更愿意牺牲平滑性来满足安全距离。
                    new_lambda2_ *= 2;

                    printf("\033[32miter(+1)=%d,time(ms)=%5.3f,keep optimizing\n\033[0m", iter_num_, time_ms);
                }
            }
            else if (result == lbfgs::LBFGSERR_CANCELED)
            {
                // earlyExit 返回非零会触发 CANCELED；此时通常表示发现新碰撞，需要 rebound 重启。
                flag_force_return = true;
                rebound_times++;
                cout << "iter=" << iter_num_ << ",time(ms)=" << time_ms << ",rebound." << endl;
            }
            else
            {
                ROS_WARN("Solver error. Return = %d, %s. Skip this planning.", result, lbfgs::lbfgs_strerror(result));
                // while (ros::ok());
            }

        } while ((flag_occ && restart_nums < MAX_RESART_NUMS_SET) ||
                 (flag_force_return && force_stop_type_ == STOP_FOR_REBOUND && rebound_times <= 20));

        return success;
    }

    bool BsplineOptimizer::refine_optimize()
    {
        iter_num_ = 0;
        // refine 同样只优化中间控制点，保留起终端控制点保证边界连续性。
        int start_id = order_;
        int end_id = this->cps_.points.cols() - order_;
        // refine 阶段变量布局与 rebound 相同：只展开中间控制点的 xyz。
        variable_num_ = 3 * (end_id - start_id);

        double q[variable_num_];
        double final_cost;

        memcpy(q, cps_.points.data() + 3 * start_id, variable_num_ * sizeof(q[0]));

        // 保存原始贴合权重，refine 失败时可能临时放大 lambda4_，退出前需要恢复。
        double origin_lambda4 = lambda4_;
        bool flag_safe = true;
        int iter_count = 0;
        do
        {
            lbfgs::lbfgs_parameter_t lbfgs_params;
            lbfgs::lbfgs_load_default_parameters(&lbfgs_params);
            lbfgs_params.mem_size = 16;
            lbfgs_params.max_iterations = 200;
            lbfgs_params.g_epsilon = 0.001;

            // refine 不使用 earlyExit，直接围绕平滑、可行性和参考线贴合求一个更规整的轨迹。
            int result = lbfgs::lbfgs_optimize(variable_num_, q, &final_cost, BsplineOptimizer::costFunctionRefine, NULL, NULL, this, &lbfgs_params);
            if (result == lbfgs::LBFGS_CONVERGENCE ||
                result == lbfgs::LBFGSERR_MAXIMUMITERATION ||
                result == lbfgs::LBFGS_ALREADY_MINIMIZED ||
                result == lbfgs::LBFGS_STOP)
            {
                // pass
            }
            else
            {
                ROS_ERROR("Solver error in refining!, return = %d, %s", result, lbfgs::lbfgs_strerror(result));
            }

            UniformBspline traj = UniformBspline(cps_.points, 3, bspline_interval_);
            double tm, tmp;
            traj.getTimeSpan(tm, tmp);
            // 优化后再次采样检查前 2/3 段轨迹是否安全。
            // 这里仍按地图分辨率换算时间采样间隔，避免两个时间采样点之间跨过障碍。
            double t_step = (tmp - tm) / ((traj.evaluateDeBoorT(tmp) - traj.evaluateDeBoorT(tm)).norm() / grid_map_->getResolution()); // Step size is defined as the maximum size that can passes throgth every gird.
            for (double t = tm; t < tmp * 2 / 3; t += t_step)
            {
                if (grid_map_->getInflateOccupancy(traj.evaluateDeBoorT(t)))
                {
                    // cout << "Refined traj hit_obs, t=" << t << " P=" << traj.evaluateDeBoorT(t).transpose() << endl;

                    Eigen::MatrixXd ref_pts(ref_pts_.size(), 3);
                    for (size_t i = 0; i < ref_pts_.size(); i++)
                    {
                        ref_pts.row(i) = ref_pts_[i].transpose();
                    }

                    flag_safe = false;
                    break;
                }
            }

            if (!flag_safe)
                // 若 refine 之后发生碰撞，尝试提高贴合参考线的权重，让轨迹更靠近安全参考路径。
                lambda4_ *= 2;

            iter_count++;
        } while (!flag_safe && iter_count <= 0);

        lambda4_ = origin_lambda4;

        // cout << "iter_num_=" << iter_num_ << endl;

        return flag_safe;
    }

    void BsplineOptimizer::combineCostRebound(const double *x, double *grad, double &f_combine, const int n)
    {

        // 把 L-BFGS 当前变量写回 cps_.points，后续各项代价都直接基于控制点矩阵计算。
        // 这里与 rebound_optimize 中的 memcpy 方向相反：q/x 是求解器变量，cps_.points 是矩阵形式。
        memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

        /* ---------- evaluate cost and gradient ---------- */
        // rebound 总代价由平滑、避障距离和动力学可行性三部分组成。
        double f_smoothness, f_distance, f_feasibility;

        Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.size);
        Eigen::MatrixXd g_distance = Eigen::MatrixXd::Zero(3, cps_.size);
        Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.size);

        calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
        calcDistanceCostRebound(cps_.points, f_distance, g_distance, iter_num_, f_smoothness);
        calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

        // 三项代价按 ROS 参数中的权重线性加权；new_lambda2_ 可能在重启时被放大。
        f_combine = lambda1_ * f_smoothness + new_lambda2_ * f_distance + lambda3_ * f_feasibility;
        // printf("origin %f %f %f %f\n", f_smoothness, f_distance, f_feasibility, f_combine);

        // 对三项梯度按权重线性组合后，再拷贝回 L-BFGS 的一维梯度数组。
        // 只拷贝中间可优化控制点对应的梯度，首尾固定控制点梯度即使计算了也不交给求解器。
        Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + new_lambda2_ * g_distance + lambda3_ * g_feasibility;
        memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
    }

    void BsplineOptimizer::combineCostRefine(const double *x, double *grad, double &f_combine, const int n)
    {

        // 将求解器变量同步到控制点矩阵；固定端点不在变量数组中。
        // order_ 个起点控制点和 order_ 个终点控制点保持不变，保证与前后轨迹段连接稳定。
        memcpy(cps_.points.data() + 3 * order_, x, n * sizeof(x[0]));

        /* ---------- evaluate cost and gradient ---------- */
        // refine 总代价由平滑、贴合参考轨迹和动力学可行性组成，不再直接使用距离场反弹约束。
        double f_smoothness, f_fitness, f_feasibility;

        Eigen::MatrixXd g_smoothness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
        Eigen::MatrixXd g_fitness = Eigen::MatrixXd::Zero(3, cps_.points.cols());
        Eigen::MatrixXd g_feasibility = Eigen::MatrixXd::Zero(3, cps_.points.cols());

        // time_satrt = ros::Time::now();

        calcSmoothnessCost(cps_.points, f_smoothness, g_smoothness);
        calcFitnessCost(cps_.points, f_fitness, g_fitness);
        calcFeasibilityCost(cps_.points, f_feasibility, g_feasibility);

        /* ---------- convert to solver format...---------- */
        // refine 中 lambda4_ 控制贴合参考轨迹的强度；若 refine 后碰撞，外层可能临时放大它。
        f_combine = lambda1_ * f_smoothness + lambda4_ * f_fitness + lambda3_ * f_feasibility;
        // printf("origin %f %f %f %f\n", f_smoothness, f_fitness, f_feasibility, f_combine);

        // Eigen 仍按列主序展开，和 L-BFGS 的 x/grad 内存布局保持一致。
        Eigen::MatrixXd grad_3D = lambda1_ * g_smoothness + lambda4_ * g_fitness + lambda3_ * g_feasibility;
        memcpy(grad, grad_3D.data() + 3 * order_, n * sizeof(grad[0]));
    }

} // namespace ego_planner
