#ifndef _UNIFORM_BSPLINE_H_
#define _UNIFORM_BSPLINE_H_

#include <Eigen/Eigen>
#include <algorithm>
#include <iostream>

using namespace std;

namespace ego_planner
{
    // An implementation of non-uniform B-spline with different dimensions
    // It also represents uniform B-spline which is a special case of non-uniform
    // 这里主要用于三维轨迹：控制点矩阵通常为 3 x N，每一列是一个控制点。
    class UniformBspline
    {
    private:
        // control points for B-spline with different dimensions.
        // Each row represents one single control point
        // The dimension is determined by column number
        // e.g. B-spline with N points in 3D space -> Nx3 matrix
        // 注意本工程实际实现里按列存控制点：行数为维度，列数为控制点数量。
        Eigen::MatrixXd control_points_;

        int p_, n_, m_;     // p degree, n+1 control points, m = n+p+1
        Eigen::VectorXd u_; // knots vector
        double interval_;   // knot span \delta t

        Eigen::MatrixXd getDerivativeControlPoints();

        // 速度/加速度约束和可行性检查时允许的容差。
        double limit_vel_, limit_acc_, limit_ratio_, feasibility_tolerance_; // physical limits and time adjustment ratio

    public:
        UniformBspline() {}
        // 用控制点、次数和均匀 knot 间隔构造 B 样条。
        UniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);
        ~UniformBspline();

        // 返回内部控制点矩阵。
        Eigen::MatrixXd get_control_points(void) { return control_points_; }

        // initialize as an uniform B-spline
        // 按均匀时间间隔生成 knot 向量，并保存控制点和次数。
        void setUniformBspline(const Eigen::MatrixXd &points, const int &order, const double &interval);

        // get / set basic bspline info

        // 手动设置 knot 向量，常用于导数 B 样条裁剪首尾 knot。
        void setKnot(const Eigen::VectorXd &knot);
        // 获取 knot 向量。
        Eigen::VectorXd getKnot();
        // 获取控制点矩阵。
        Eigen::MatrixXd getControlPoint();
        // 获取均匀 knot 间隔。
        double getInterval();
        // 获取 B 样条的有效 knot 参数区间 [u_p, u_{m-p}]：
        // - p 是曲线次数，m 是最后一个 knot 的索引；
        // - de Boor 求值时需要左右各有足够 knot/控制点支撑，所以真正可评价的区间
        //   不是整个 knot 向量 [u_0, u_m]，而是去掉首尾 p 个 knot 后的 [u_p, u_{m-p}]；
        // - 在本工程默认构造中 u_p 通常等于 0，u_{m-p} 对应轨迹结束的绝对 knot 参数。
        bool getTimeSpan(double &um, double &um_p);

        // compute position / derivative

        // 使用 de Boor 算法在 knot 参数 u 处求曲线点。
        // u 是 B 样条数学参数，合法范围为 [u_p, u_{m-p}]，不是“从 0 开始的轨迹时间”。
        Eigen::VectorXd evaluateDeBoor(const double &u); // use u \in [up, u_mp]
        // 使用从 0 开始的轨迹时间 t 求曲线点，内部会平移到 knot 参数 u。
        // evaluateDeBoorT(t) 中的 t 表示相对轨迹时间，范围为 [0, duration]；
        // evaluateDeBoor() 需要的是绝对 knot 参数 u，所以要做转换：
        //   u = t + u_(p_)
        // 也就是把 t=0 对齐到有效区间起点 u_p，把 t=duration 对齐到 u_{m-p}。
        inline Eigen::VectorXd evaluateDeBoorT(const double &t) { return evaluateDeBoor(t + u_(p_)); } // use t \in [0, duration]
        // 返回当前 B 样条的一阶导数曲线。
        UniformBspline getDerivative();

        // 3D B-spline interpolation of points in point_set, with boundary vel&acc
        // constraints
        // input : (K+2) points with boundary vel/acc; ts
        // output: (K+6) control_pts
        // 根据离散路径点和起终点速度/加速度，反解三次均匀 B 样条控制点。
        static void parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                          const vector<Eigen::Vector3d> &start_end_derivative,
                                          Eigen::MatrixXd &ctrl_pts);

        /* check feasibility, adjust time */

        // 设置速度、加速度物理上限和可行性检查容差。
        void setPhysicalLimits(const double &vel, const double &acc, const double &tolerance);
        // 检查当前控制点导出的速度/加速度包络是否满足约束，并给出建议时间缩放比例。
        bool checkFeasibility(double &ratio, bool show = false);
        // 按 ratio 拉长 knot 时间，降低速度和加速度。
        void lengthenTime(const double &ratio);

        /* for performance evaluation */

        // 获取轨迹总时长。
        double getTimeSum();
        // 按采样分辨率近似计算轨迹弧长。
        double getLength(const double &res = 0.01);
        // 近似计算 jerk 积分指标，用于评价轨迹平滑性。
        double getJerk();
        // 采样统计平均速度和最大速度。
        void getMeanAndMaxVel(double &mean_v, double &max_v);
        // 采样统计平均加速度和最大加速度。
        void getMeanAndMaxAcc(double &mean_a, double &max_a);

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
} // namespace ego_planner
#endif
