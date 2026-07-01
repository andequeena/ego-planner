#ifndef _BSPLINE_OPTIMIZER_H_
#define _BSPLINE_OPTIMIZER_H_

#include <Eigen/Eigen>
#include <path_searching/dyn_a_star.h>
#include <bspline_opt/uniform_bspline.h>
#include <plan_env/grid_map.h>
#include <ros/ros.h>
#include "bspline_opt/lbfgs.hpp"

// Gradient and elasitc band optimization

// Input: a signed distance field and a sequence of points
// Output: the optimized sequence of points
// The format of points: N x 3 matrix, each row is a point
namespace ego_planner
{

    // ControlPoints 保存优化过程中围绕 B 样条控制点产生的辅助信息。
    // points 是实际控制点；base_point/direction 用于构造避障反弹代价。
    class ControlPoints
    {
    public:
        // clearance 是控制点需要保持的最小安全距离。
        double clearance;
        // size 是控制点数量，等于 points 的列数。
        int size;
        // points 为 3 x size，每一列是一个三维 B 样条控制点。
        Eigen::MatrixXd points;
        std::vector<std::vector<Eigen::Vector3d>> base_point; // The point at the statrt of the direction vector (collision point)
        std::vector<std::vector<Eigen::Vector3d>> direction;  // Direction vector, must be normalized.
        std::vector<bool> flag_temp;                          // A flag that used in many places. Initialize it everytime before using it.
        // std::vector<bool> occupancy;

        void resize(const int size_set)
        {
            // resize 会重置所有与控制点数量相关的缓存，避免沿用上一条轨迹的碰撞信息。
            size = size_set;

            base_point.clear();
            direction.clear();
            flag_temp.clear();
            // occupancy.clear();

            // points 固定为三维轨迹控制点；每个控制点都可能对应多个障碍反弹约束。
            points.resize(3, size_set);
            base_point.resize(size);
            direction.resize(size);
            flag_temp.resize(size);
            // occupancy.resize(size);
        }
    };

    class BsplineOptimizer
    {

    public:
        BsplineOptimizer() {}
        ~BsplineOptimizer() {}

        /* main API */
        // 设置地图环境；优化器通过 GridMap 查询膨胀占据、分辨率和距离场信息。
        void setEnvironment(const GridMap::Ptr &env);
        // 从 ROS 参数服务器读取优化权重、最大速度/加速度、安全距离等参数。
        void setParam(ros::NodeHandle &nh);
        // 旧版统一入口：给定初始控制点、时间间隔和代价类型，返回优化后的控制点。
        Eigen::MatrixXd BsplineOptimizeTraj(const Eigen::MatrixXd &points, const double &ts,
                                            const int &cost_function, int max_num_id, int max_time_id);

        /* helper function */

        // required inputs
        // 设置待优化控制点矩阵。
        void setControlPoints(const Eigen::MatrixXd &points);
        // 设置均匀 B 样条的时间间隔，影响速度/加速度尺度。
        void setBsplineInterval(const double &ts);
        // 设置代价函数组合类型，供旧版 optimize() 路径使用。
        void setCostFunction(const int &cost_function);
        // 设置迭代次数和时间相关的终止条件。
        void setTerminateCond(const int &max_num_id, const int &max_time_id);

        // optional inputs
        // 设置几何引导路径，用于让轨迹沿参考路径靠拢。
        void setGuidePath(const vector<Eigen::Vector3d> &guide_pt);
        // 设置必须经过或靠近的航点及其对应控制点索引，最多约束中间 N-2 个点。
        void setWaypoints(const vector<Eigen::Vector3d> &waypts,
                          const vector<int> &waypt_idx); // N-2 constraints at most

        // 执行旧版通用优化流程。
        void optimize();

        // 返回当前内部保存的控制点矩阵。
        Eigen::MatrixXd getControlPoints();

        // rebound 阶段用于局部绕障搜索的 A* 对象。
        AStar::Ptr a_star_;
        // refine 阶段用于曲线贴合的参考点序列。
        std::vector<Eigen::Vector3d> ref_pts_;

        // 根据初始轨迹和障碍物切分碰撞段，生成每个碰撞段的 A* 绕障路径与反弹方向。
        std::vector<std::vector<Eigen::Vector3d>> initControlPoints(Eigen::MatrixXd &init_points, bool flag_first_init = true);
        // 基于 initControlPoints() 生成的约束进行 rebound 优化，必须在其之后调用。
        bool BsplineOptimizeTrajRebound(Eigen::MatrixXd &optimal_points, double ts); // must be called after initControlPoints()
        // 对 rebound 后的轨迹进行二次优化，使其更平滑并贴合参考轨迹。
        bool BsplineOptimizeTrajRefine(const Eigen::MatrixXd &init_points, const double ts, Eigen::MatrixXd &optimal_points);

        // 暴露 B 样条次数，供外部模块确定首尾固定控制点数量。
        inline int getOrder(void) { return order_; }

    private:
        // 地图环境指针，负责碰撞检测和距离查询。
        GridMap::Ptr grid_map_;

        // L-BFGS 早停原因：正常继续、发现新碰撞需要 rebound、或出现不可恢复错误。
        enum FORCE_STOP_OPTIMIZE_TYPE
        {
            DONT_STOP,
            STOP_FOR_REBOUND,
            STOP_FOR_ERROR
        } force_stop_type_;

        // main input
        // Eigen::MatrixXd control_points_;     // B-spline control points, N x dim
        double bspline_interval_; // B-spline knot span
        Eigen::Vector3d end_pt_;  // end of the trajectory
        // int             dim_;                // dimension of the B-spline
        //
        vector<Eigen::Vector3d> guide_pts_; // geometric guiding path points, N-6
        vector<Eigen::Vector3d> waypoints_; // waypts constraints
        vector<int> waypt_idx_;             // waypts constraints index
                                            //
        int max_num_id_, max_time_id_;      // stopping criteria
        int cost_function_;                 // used to determine objective function
        double start_time_;                 // global time for moving obstacles

        /* optimization parameters */
        // 三次 B 样条通常 order_ 为 3。
        int order_;                    // bspline degree
        double lambda1_;               // jerk smoothness weight
        double lambda2_, new_lambda2_; // distance weight
        double lambda3_;               // feasibility weight
        double lambda4_;               // curve fitting

        int a;
        //
        double dist0_;             // safe distance
        double max_vel_, max_acc_; // dynamic limits

        int variable_num_;              // optimization variables
        int iter_num_;                  // iteration of the solver
        // 记录历史最优变量，供旧版优化流程使用。
        Eigen::VectorXd best_variable_; //
        // 记录历史最小代价，供重启或早停时参考。
        double min_cost_;               //

        // 当前轨迹控制点及其避障辅助约束。
        ControlPoints cps_;

        /* cost function */
        /* calculate each part of cost function with control points q as input */

        // 旧版优化器回调：把一维变量 x 转换为控制点并计算总代价和梯度。
        static double costFunction(const std::vector<double> &x, std::vector<double> &grad, void *func_data);
        // 旧版总代价组合函数。
        void combineCost(const std::vector<double> &x, vector<double> &grad, double &cost);

        // q contains all control points
        // 平滑代价：默认使用三阶差分 jerk，也可切换为二阶差分加速度。
        void calcSmoothnessCost(const Eigen::MatrixXd &q, double &cost,
                                Eigen::MatrixXd &gradient, bool falg_use_jerk = true);
        // 动力学可行性代价：惩罚超过最大速度/最大加速度的控制点差分。
        void calcFeasibilityCost(const Eigen::MatrixXd &q, double &cost,
                                 Eigen::MatrixXd &gradient);
        // rebound 避障距离代价：根据 base_point/direction 把控制点推出障碍。
        void calcDistanceCostRebound(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient, int iter_num, double smoothness_cost);
        // refine 贴合代价：让 B 样条曲线靠近 ref_pts_。
        void calcFitnessCost(const Eigen::MatrixXd &q, double &cost, Eigen::MatrixXd &gradient);
        // 优化中再次碰撞检查；若发现新碰撞，则生成新的 rebound 约束并触发早停。
        bool check_collision_and_rebound(void);

        // L-BFGS 进度回调：返回非零可取消当前求解。
        static int earlyExit(void *func_data, const double *x, const double *g, const double fx, const double xnorm, const double gnorm, const double step, int n, int k, int ls);
        // L-BFGS rebound 阶段目标函数回调。
        static double costFunctionRebound(void *func_data, const double *x, double *grad, const int n);
        // L-BFGS refine 阶段目标函数回调。
        static double costFunctionRefine(void *func_data, const double *x, double *grad, const int n);

        // 执行 rebound/refine 两个具体优化阶段。
        bool rebound_optimize();
        bool refine_optimize();
        // rebound 阶段总代价组合：smoothness + distance + feasibility。
        void combineCostRebound(const double *x, double *grad, double &f_combine, const int n);
        // refine 阶段总代价组合：smoothness + fitness + feasibility。
        void combineCostRefine(const double *x, double *grad, double &f_combine, const int n);

        /* for benckmark evaluation only */
    public:
        typedef unique_ptr<BsplineOptimizer> Ptr;

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };

} // namespace ego_planner
#endif
