#include "bspline_opt/uniform_bspline.h"
#include <ros/ros.h>

namespace ego_planner
{
    UniformBspline::UniformBspline(const Eigen::MatrixXd &points, const int &order,
                                   const double &interval)
    {
        // 构造时直接按给定控制点、阶数和时间间隔初始化均匀 B 样条。
        // points: 3 x N 或 dim x N，列是控制点 P0, P1, ..., PN-1。
        // order: 这里传入的是 degree（次数）p，工程里常用三次 B 样条，所以 order=3。
        // interval: 相邻 knot 的时间间隔，也就是轨迹离散时间步长 dt。
        setUniformBspline(points, order, interval);
    }

    UniformBspline::~UniformBspline() {}

    void UniformBspline::setUniformBspline(const Eigen::MatrixXd &points, const int &order,
                                           const double &interval)
    {
        // control_points_ 每一列是一个控制点：
        //   control_points_.col(i) = P_i = [x_i, y_i, z_i]^T
        // p_ 是 B 样条次数（degree），不是控制点数量；三次 B 样条 p_=3。
        control_points_ = points;
        p_ = order;
        interval_ = interval;

        // n_ 表示最后一个控制点索引，所以控制点总数为 n_ + 1。
        // 例如 points 有 8 列控制点，则 n_=7，控制点索引是 P0 ... P7。
        //
        // m_ 表示最后一个 knot 索引，knot 总数为 m_ + 1。
        // B 样条标准关系：
        //   控制点个数 = n + 1
        //   次数 = p
        //   knot 个数 = m + 1
        //   m = n + p + 1
        // 所以这里 m_ = n_ + p_ + 1。
        n_ = points.cols() - 1;
        m_ = n_ + p_ + 1;

        // knot 向量长度为 m_ + 1。这里构造的是等间隔 knot。
        // knot 向量可以理解为 B 样条的“参数时间轴”：
        //   u_ = [u0, u1, ..., um]
        // 对均匀 B 样条，相邻 knot 间隔恒为 interval_。
        //
        // B 样条真正可评价的 knot 参数区间是 [u_p, u_{m-p}]：
        // 首尾各去掉 p 个 knot，是因为 p 次 de Boor 求值需要足够的局部 knot/控制点支撑。
        //
        // 本工程把 u_p 设计成 0：
        //   当 i=p 时，u_(p) = (-p+p)*interval = 0
        // 这样外部用 evaluateDeBoorT(t) 时，t=0 就对应曲线起点参数 u_p。
        //
        // 以 p=3、interval=0.1 为例，前几个 knot 是：
        //   u0=-0.3, u1=-0.2, u2=-0.1, u3=0.0, u4=0.1 ...
        // 有效求值从 u3=0.0 开始。
        u_ = Eigen::VectorXd::Zero(m_ + 1);
        for (int i = 0; i <= m_; ++i)
        {
            if (i <= p_)
            {
                // 前 p+1 个 knot 从负时间开始，便于 evaluateDeBoorT(t) 以 t=0 对应 u_p。
                // 注意这里不是为了让曲线在负时间运行，而是为了构造完整的 knot 向量。
                // 有效轨迹仍然从 u_p 开始；u0...u_{p-1} 是 de Boor 计算边界时需要的支撑。
                u_(i) = double(-p_ + i) * interval_;
            }
            else if (i > p_ && i <= m_ - p_)
            {
                // 中间有效区间保持固定时间步长 interval_。
                // 这些 knot 对应轨迹主体部分，evaluateDeBoor() 大部分时候都落在这里。
                u_(i) = u_(i - 1) + interval_;
            }
            else if (i > m_ - p_)
            {
                // 尾部继续等间隔延伸，使导数和边界计算仍有完整 knot 支撑。
                // 和开头一样，u_{m-p+1}...u_m 主要用于末端的局部递推支撑。
                u_(i) = u_(i - 1) + interval_;
            }
        }
    }

    void UniformBspline::setKnot(const Eigen::VectorXd &knot)
    {
        // 允许外部直接替换 knot 向量。
        // 典型用途在 getDerivative()：导数 B 样条的 knot 需要去掉原曲线首尾各一个 knot。
        this->u_ = knot;
    }

    Eigen::VectorXd UniformBspline::getKnot() { return this->u_; }

    bool UniformBspline::getTimeSpan(double &um, double &um_p)
    {
        // 有效曲线定义在 [u_p, u_{m-p}]，这里返回的是 B 样条的绝对 knot 参数范围。
        // 若想用从 0 开始的轨迹时间 t，则 t = u - u_p，反过来 u = t + u_p。
        //
        // 参数名 um/um_p 来自原作者写法，可以读成：
        //   um   <- u_p      有效起点
        //   um_p <- u_{m-p}  有效终点
        if (p_ > u_.rows() || m_ - p_ > u_.rows())
            return false;

        um = u_(p_);
        um_p = u_(m_ - p_);

        return true;
    }

    Eigen::MatrixXd UniformBspline::getControlPoint() { return control_points_; }

    Eigen::VectorXd UniformBspline::evaluateDeBoor(const double &u)
    {
        // 将输入参数限制在有效 knot 区间内，避免越界访问控制点。
        // 如果传入 u 小于起点，就按起点算；如果大于终点，就按终点算。
        double ub = min(max(u_(p_), u), u_(m_ - p_));

        // determine which [ui,ui+1] lay in
        // 找到 ub 所在的 knot 区间 [u_k, u_{k+1}]。
        // k 的意义：当前参数 ub 落在第 k 段 knot span。
        // 对 p 次 B 样条，落在 [u_k, u_{k+1}] 时，只会受到 P_{k-p}, P_{k-p+1}, ..., P_k 这 p+1 个控制点影响。
        int k = p_;
        while (true)
        {
            if (u_(k + 1) >= ub)
                break;
            ++k;
        }

        /* deBoor's alg */
        // d 初始为影响当前区间的 p+1 个控制点。
        // d[0] 对应 P_{k-p}，d[p] 对应 P_k。
        // 后面会不断在这些点之间做线性插值，最后收缩成一个曲线点。
        vector<Eigen::VectorXd> d;
        for (int i = 0; i <= p_; ++i)
        {
            d.push_back(control_points_.col(k - p_ + i));
            // cout << d[i].transpose() << endl;
        }

        // de Boor 递推：逐层线性插值，最终 d[p_] 即曲线在 ub 的取值。
        //
        // r 表示递推层数：
        //   r=1: 用原始控制点两两插值得到第一层点；
        //   r=2: 再对第一层点插值；
        //   ...
        //   r=p: 得到最终曲线点。
        //
        // i 从后往前更新，是为了原地复用 d 数组时不覆盖下一步还要用的 d[i-1]。
        for (int r = 1; r <= p_; ++r)
        {
            for (int i = p_; i >= r; --i)
            {
                // alpha 是当前 ub 在相关 knot 区间里的归一化比例，范围通常在 [0,1]。
                // 它决定插值更靠近左侧点 d[i-1] 还是右侧点 d[i]。
                //
                // 公式来自 de Boor：
                //   d_i^r = (1-alpha) * d_{i-1}^{r-1} + alpha * d_i^{r-1}
                //
                // 代码里的索引 i + k - p_、i + 1 + k - r
                // 是把局部编号 d[i] 映射回全局 knot 向量 u_ 的编号。
                double alpha = (ub - u_[i + k - p_]) / (u_[i + 1 + k - r] - u_[i + k - p_]);
                // cout << "alpha: " << alpha << endl;
                d[i] = (1 - alpha) * d[i - 1] + alpha * d[i];
            }
        }
        return d[p_];
    }

    Eigen::MatrixXd UniformBspline::getDerivativeControlPoints()
    {
        // The derivative of a b-spline is also a b-spline, its order become p_-1
        // control point Qi = p_*(Pi+1-Pi)/(ui+p_+1-ui+1)
        // 对原控制点做一阶差分，并按 knot 间隔缩放，即可得到导数曲线的控制点。
        //
        // 原曲线：
        //   C(u) = sum_i P_i * N_{i,p}(u)
        // 一阶导数仍然是 B 样条：
        //   C'(u) = sum_i Q_i * N_{i,p-1}(u)
        // 其中导数控制点：
        //   Q_i = p * (P_{i+1} - P_i) / (u_{i+p+1} - u_{i+1})
        //
        // 为什么分母是 u_{i+p+1} - u_{i+1}？
        // P_{i+1} - P_i 表示相邻两个位置控制点的空间差分，但 B 样条不是折线，
        // 这个差分并不对应简单的 knot 区间 [u_i, u_{i+1}]。
        // 根据 B 样条基函数求导公式，把相邻项合并成 N_{i,p-1}(u) 后，
        // Q_i 的时间尺度由导数基函数的支撑区间 [u_{i+1}, u_{i+p+1}] 决定，
        // 所以要除以这段参数时间跨度：
        //   u_{i+p+1} - u_{i+1}
        //
        // 对均匀 knot，分母基本就是 p * interval_，所以 Q_i 近似就是
        // 相邻控制点差分除以时间，代表速度控制点。
        Eigen::MatrixXd ctp(control_points_.rows(), control_points_.cols() - 1);
        for (int i = 0; i < ctp.cols(); ++i)
        {
            // ctp 的列数比原控制点少 1，因为相邻两个 P 才能生成一个导数控制点 Q。
            ctp.col(i) =
                p_ * (control_points_.col(i + 1) - control_points_.col(i)) / (u_(i + p_ + 1) - u_(i + 1));
        }
        return ctp;
    }

    UniformBspline UniformBspline::getDerivative()
    {
        // 构造导数 B 样条：次数降低 1，控制点改为差分后的导数控制点。
        // 如果原曲线是位置曲线，那么第一次 getDerivative() 得到速度曲线；
        // 再调用一次得到加速度曲线；第三次得到 jerk 曲线。
        Eigen::MatrixXd ctp = getDerivativeControlPoints();
        UniformBspline derivative(ctp, p_ - 1, interval_);

        /* cut the first and last knot */
        // 导数曲线的 knot 向量比原曲线少首尾各一个 knot。
        // 原因：p 次 B 样条求导后变成 p-1 次 B 样条，
        // 对应的基函数定义域也会收缩一层，所以 knot 取 u_ 的中间段。
        Eigen::VectorXd knot(u_.rows() - 2);
        knot = u_.segment(1, u_.rows() - 2);
        derivative.setKnot(knot);

        return derivative;
    }

    double UniformBspline::getInterval() { return interval_; }

    void UniformBspline::setPhysicalLimits(const double &vel, const double &acc, const double &tolerance)
    {
        // 保存动力学约束，用于后续检查速度、加速度是否超过限制。
        limit_vel_ = vel;
        limit_acc_ = acc;
        limit_ratio_ = 1.1;
        feasibility_tolerance_ = tolerance;
    }

    bool UniformBspline::checkFeasibility(double &ratio, bool show)
    {
        // fea 表示当前曲线是否满足速度和加速度限制；ratio 给出建议的时间放大倍率。
        bool fea = true;

        Eigen::MatrixXd P = control_points_;
        int dimension = control_points_.rows();

        /* check vel feasibility and insert points */
        // 一阶导数控制点给出了每段速度包络，若任一维超过限制则判为不可行。
        double max_vel = -1.0;
        double enlarged_vel_lim = limit_vel_ * (1.0 + feasibility_tolerance_) + 1e-4;
        for (int i = 0; i < P.cols() - 1; ++i)
        {
            Eigen::VectorXd vel = p_ * (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1));

            if (fabs(vel(0)) > enlarged_vel_lim || fabs(vel(1)) > enlarged_vel_lim ||
                fabs(vel(2)) > enlarged_vel_lim)
            {

                if (show)
                    cout << "[Check]: Infeasible vel " << i << " :" << vel.transpose() << endl;
                fea = false;

                // 记录所有维度中的最大速度超限值，用于估计时间拉伸比例。
                for (int j = 0; j < dimension; ++j)
                {
                    max_vel = max(max_vel, fabs(vel(j)));
                }
            }
        }

        /* acc feasibility */
        // 二阶导数控制点给出了加速度包络，计算方式等价于对速度控制点再求导。
        double max_acc = -1.0;
        double enlarged_acc_lim = limit_acc_ * (1.0 + feasibility_tolerance_) + 1e-4;
        for (int i = 0; i < P.cols() - 2; ++i)
        {
            // 这行是在直接计算“加速度 B 样条”的控制点 A_i。
            // 可以分两步理解，不要一上来硬看完整长公式：
            //
            // 1. 先看速度控制点。由位置控制点 P_i 求导可得：
            //      V_i     = p * (P_{i+1} - P_i)     / (u_{i+p+1} - u_{i+1})
            //      V_{i+1} = p * (P_{i+2} - P_{i+1}) / (u_{i+p+2} - u_{i+2})
            //
            // 2. 加速度就是对速度曲线再求一次导。
            //    速度曲线的次数已经从 p 降为 p-1，所以第二次求导会多出系数 (p-1)：
            //      A_i = (p-1) * (V_{i+1} - V_i) / (u_{i+p+1} - u_{i+2})
            //
            // 把上面的 V_i 和 V_{i+1} 代入 A_i，就得到代码里的展开式：
            //      A_i = p * (p-1) *
            //            ( (P_{i+2}-P_{i+1})/(u_{i+p+2}-u_{i+2})
            //             -(P_{i+1}-P_i)    /(u_{i+p+1}-u_{i+1}) )
            //            /(u_{i+p+1}-u_{i+2})
            //
            // 直觉上：
            //   (P_{i+1}-P_i) / 时间       是一段速度趋势；
            //   两段速度趋势相减 / 时间    就是加速度趋势。
            //
            // 如果 knot 是均匀的，三次 B 样条 p=3 时：
            //   u_{i+p+2}-u_{i+2} = 3*dt
            //   u_{i+p+1}-u_{i+1} = 3*dt
            //   u_{i+p+1}-u_{i+2} = 2*dt
            // 所以整体会化简成类似：
            //   A_i = (P_{i+2} - 2P_{i+1} + P_i) / dt^2
            // 这就是普通离散二阶差分“位置 -> 加速度”的形式。
            Eigen::VectorXd acc = p_ * (p_ - 1) *
                                  ((P.col(i + 2) - P.col(i + 1)) / (u_(i + p_ + 2) - u_(i + 2)) -
                                   (P.col(i + 1) - P.col(i)) / (u_(i + p_ + 1) - u_(i + 1))) /
                                  (u_(i + p_ + 1) - u_(i + 2));

            if (fabs(acc(0)) > enlarged_acc_lim || fabs(acc(1)) > enlarged_acc_lim ||
                fabs(acc(2)) > enlarged_acc_lim)
            {

                if (show)
                    cout << "[Check]: Infeasible acc " << i << " :" << acc.transpose() << endl;
                fea = false;

                // 记录最大加速度超限值；时间缩放后加速度按 1/t^2 缩小。
                for (int j = 0; j < dimension; ++j)
                {
                    max_acc = max(max_acc, fabs(acc(j)));
                }
            }
        }

        // 速度随时间缩放 ratio 按 1/ratio 变化，加速度按 1/ratio^2 变化。
        ratio = max(max_vel / limit_vel_, sqrt(fabs(max_acc) / limit_acc_));

        return fea;
    }

    void UniformBspline::lengthenTime(const double &ratio)
    {
        // 只拉伸中间有效 knot 区间，首尾若干 knot 保持相对结构，避免边界形状突变。
        int num1 = 5;
        int num2 = getKnot().rows() - 1 - 5;

        // delta_t 是总时长增加量，t_inc 是分摊到每个 knot 间隔上的增量。
        double delta_t = (ratio - 1.0) * (u_(num2) - u_(num1));
        double t_inc = delta_t / double(num2 - num1);
        for (int i = num1 + 1; i <= num2; ++i)
            u_(i) += double(i - num1) * t_inc;
        for (int i = num2 + 1; i < u_.rows(); ++i)
            u_(i) += delta_t;
    }

    // void UniformBspline::recomputeInit() {}

    // 根据给定的离散点集（point_set）及起止导数信息，生成均匀三次 B 样条的控制点（ctrl_pts）。
    // 时间步长（或均匀参数间隔），用于计算速度/加速度约束。
    // 要求 B 样条通过的离散点集合（轨迹采样点）。
    // 起点/终点速度和加速度信息，顺序通常是 [start_vel, end_vel, start_acc, end_acc]。
    // 输出的控制点矩阵，3×(K+2)，每列对应一个控制点的 x、y、z 坐标。
    void UniformBspline::parameterizeToBspline(const double &ts, const vector<Eigen::Vector3d> &point_set,
                                               const vector<Eigen::Vector3d> &start_end_derivative,
                                               Eigen::MatrixXd &ctrl_pts)
    {
        // ts 是离散点之间的时间间隔，必须大于 0 才能建立速度/加速度约束。
        if (ts <= 0)
        {
            cout << "[B-spline]:time step error." << endl;
            return;
        }

        // 至少需要 4 个点才能生成三次 B 样条，否则约束不足。
        if (point_set.size() <= 3)
        {
            cout << "[B-spline]:point set have only " << point_set.size() << " points." << endl;
            return;
        }

        // 必须提供起点/终点速度和加速度各一个，否则边界条件不完整。
        if (start_end_derivative.size() != 4)
        {
            cout << "[B-spline]:derivatives error." << endl;
        }

        int K = point_set.size();

        // write A
        // prow/vrow/arow 分别是三次均匀 B 样条的位置、速度、加速度模板系数。
        Eigen::Vector3d prow(3), vrow(3), arow(3);
        /*
         * 这里的核心目的是：根据给定的离散路径点 point_set，以及起点/终点的速度、加速度约束，
         * 反求一组均匀三次 B 样条控制点 ctrl_pts。
         *
         * 对均匀三次 B 样条来说，曲线并不是直接穿过控制点，而是由相邻控制点加权得到。
         * 在每一个采样时刻 t_i，对应的位置、速度、加速度可以写成相邻三个控制点的线性组合：
         *
         *   位置： p_i = (P_i + 4P_{i+1} + P_{i+2}) / 6
         *   速度： v_i = (-P_i + P_{i+2}) / (2 * ts)
         *   加速度： a_i = (P_i - 2P_{i+1} + P_{i+2}) / (ts^2)
         *
         * 其中：
         *   P_i, P_{i+1}, P_{i+2} 是相邻的三个 B 样条控制点；
         *   ts 是相邻节点之间的时间间隔；
         *   [1, 4, 1] / 6 是位置插值系数；
         *   [-1, 0, 1] / (2 * ts) 是速度有限差分系数；
         *   [1, -2, 1] / (ts^2) 是加速度有限差分系数。
         *
         * 因此，下面的 prow、vrow、arow 分别就是位置、速度、加速度的模板系数。
         *
         * 举例说明：
         * 假设一共有 K 个路径采样点 q0, q1, q2, ..., q(K-1)，
         * 均匀三次 B 样条需要 K + 2 个控制点：
         *
         *   P0, P1, P2, ..., P(K+1)
         *
         * 第 0 个采样点由 P0、P1、P2 共同决定：
         *
         *   q0 = (P0 + 4P1 + P2) / 6
         *
         * 第 1 个采样点由 P1、P2、P3 共同决定：
         *
         *   q1 = (P1 + 4P2 + P3) / 6
         *
         * 第 i 个采样点由 P_i、P_{i+1}、P_{i+2} 共同决定：
         *
         *   q_i = (P_i + 4P_{i+1} + P_{i+2}) / 6
         *
         * 这些关系都可以写进矩阵 A 中。例如 K = 3 时，位置约束部分为：
         *
         *   [1/6  4/6  1/6   0     0  ] [P0]   [q0]
         *   [ 0   1/6  4/6  1/6    0  ] [P1] = [q1]
         *   [ 0    0   1/6  4/6   1/6 ] [P2]   [q2]
         *                                  [P3]
         *                                  [P4]
         *
         * 但仅有 K 个位置方程，而未知控制点有 K + 2 个，因此还需要额外约束才能唯一确定控制点。
         * 这里额外加入 4 个边界约束：
         *
         *   起点速度：     v_start = (-P0 + P2) / (2 * ts)
         *   终点速度：     v_end   = (-P(K-1) + P(K+1)) / (2 * ts)
         *   起点加速度：   a_start = (P0 - 2P1 + P2) / (ts^2)
         *   终点加速度：   a_end   = (P(K-1) - 2P(K) + P(K+1)) / (ts^2)
         *
         * 所以矩阵 A 的总行数为 K + 4：
         *   前 K 行：位置约束；
         *   第 K、K+1 行：起点和终点速度约束；
         *   第 K+2、K+3 行：起点和终点加速度约束。
         *
         * 矩阵 A 的列数为 K + 2，对应要求解的 K + 2 个控制点。
         *
         * 注意：这里的每个控制点本质上都是三维向量，即包含 x、y、z 三个坐标。
         * 为了计算方便，代码把同一个矩阵 A 分别用于 x、y、z 三个方向：
         *
         *   A * px = bx
         *   A * py = by
         *   A * pz = bz
         *
         * 其中 px、py、pz 分别是所有控制点在 x/y/z 方向上的坐标。
         * 最后再把三个方向的结果合并成 ctrl_pts，得到完整的三维控制点矩阵。
         */
        prow << 1, 4, 1;
        vrow << -1, 0, 1;
        arow << 1, -2, 1;

        // K 个位置约束 + 4 个速度/加速度约束
        // 均匀三次 B 样条，控制点数比样本点多 2 个
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(K + 4, K + 2);

        for (int i = 0; i < K; ++i)
            // 每个采样点由相邻三个控制点按 [1,4,1]/6 线性组合得到。
            A.block(i, i, 1, 3) = (1 / 6.0) * prow.transpose();

        // K 行和 K+1 行：速度约束，分别对应起点速度和终点速度。
        A.block(K, 0, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();
        A.block(K + 1, K - 1, 1, 3) = (1 / 2.0 / ts) * vrow.transpose();

        // K+2 行和 K+3 行：加速度约束，分别对应起点加速度和终点加速度。
        A.block(K + 2, 0, 1, 3) = (1 / ts / ts) * arow.transpose();
        A.block(K + 3, K - 1, 1, 3) = (1 / ts / ts) * arow.transpose();

        // 注意起点约束系数作用在控制点 [0,1,2]，终点约束系数作用在最后三个控制点 [K-1,K,K+1]。
        // cout << "A" << endl << A << endl << endl;

        // write b
        // b 的前三组分别存放 x/y/z 方向的采样点和边界导数约束。
        Eigen::VectorXd bx(K + 4), by(K + 4), bz(K + 4);
        for (int i = 0; i < K; ++i)
        {
            bx(i) = point_set[i](0);
            by(i) = point_set[i](1);
            bz(i) = point_set[i](2);
        }

        for (int i = 0; i < 4; ++i)
        {
            bx(K + i) = start_end_derivative[i](0);
            by(K + i) = start_end_derivative[i](1);
            bz(K + i) = start_end_derivative[i](2);
        }

        // solve Ax = b
        // A 通常不是简单的对角矩阵，使用列主元 QR 分解分别求解三个坐标方向。
        Eigen::VectorXd px = A.colPivHouseholderQr().solve(bx);
        Eigen::VectorXd py = A.colPivHouseholderQr().solve(by);
        Eigen::VectorXd pz = A.colPivHouseholderQr().solve(bz);

        // convert to control pts
        // 求解结果按行写回控制点矩阵：第 0/1/2 行分别是 x/y/z。
        ctrl_pts.resize(3, K + 2);
        ctrl_pts.row(0) = px.transpose();
        ctrl_pts.row(1) = py.transpose();
        ctrl_pts.row(2) = pz.transpose();

        // cout << "[B-spline]: parameterization ok." << endl;
    }

    double UniformBspline::getTimeSum()
    {
        // 返回有效曲线持续时间；若 knot 配置非法则返回 -1。
        double tm, tmp;
        if (getTimeSpan(tm, tmp))
            return tmp - tm;
        else
            return -1.0;
    }

    double UniformBspline::getLength(const double &res)
    {
        // 按固定时间分辨率采样曲线，用相邻采样点折线长度近似真实弧长。
        double length = 0.0;
        double dur = getTimeSum();
        Eigen::VectorXd p_l = evaluateDeBoorT(0.0), p_n;
        for (double t = res; t <= dur + 1e-4; t += res)
        {
            p_n = evaluateDeBoorT(t);
            length += (p_n - p_l).norm();
            p_l = p_n;
        }
        return length;
    }

    double UniformBspline::getJerk()
    {
        // jerk 是三阶导数，这里先连续求三次导数得到 jerk 曲线。
        UniformBspline jerk_traj = getDerivative().getDerivative().getDerivative();

        Eigen::VectorXd times = jerk_traj.getKnot();
        Eigen::MatrixXd ctrl_pts = jerk_traj.getControlPoint();
        int dimension = ctrl_pts.rows();

        // 对每个 jerk 控制点的平方范数按对应 knot 区间积分近似。
        double jerk = 0.0;
        for (int i = 0; i < ctrl_pts.cols(); ++i)
        {
            for (int j = 0; j < dimension; ++j)
            {
                jerk += (times(i + 1) - times(i)) * ctrl_pts(j, i) * ctrl_pts(j, i);
            }
        }

        return jerk;
    }

    void UniformBspline::getMeanAndMaxVel(double &mean_v, double &max_v)
    {
        // 通过一阶导数曲线按 0.01s 采样，统计速度模长的均值和最大值。
        UniformBspline vel = getDerivative();
        double tm, tmp;
        vel.getTimeSpan(tm, tmp);

        double max_vel = -1.0, mean_vel = 0.0;
        int num = 0;
        for (double t = tm; t <= tmp; t += 0.01)
        {
            Eigen::VectorXd vxd = vel.evaluateDeBoor(t);
            double vn = vxd.norm();

            mean_vel += vn;
            ++num;
            if (vn > max_vel)
            {
                max_vel = vn;
            }
        }

        mean_vel = mean_vel / double(num);
        mean_v = mean_vel;
        max_v = max_vel;
    }

    void UniformBspline::getMeanAndMaxAcc(double &mean_a, double &max_a)
    {
        // 通过二阶导数曲线按 0.01s 采样，统计加速度模长的均值和最大值。
        UniformBspline acc = getDerivative().getDerivative();
        double tm, tmp;
        acc.getTimeSpan(tm, tmp);

        double max_acc = -1.0, mean_acc = 0.0;
        int num = 0;
        for (double t = tm; t <= tmp; t += 0.01)
        {
            Eigen::VectorXd axd = acc.evaluateDeBoor(t);
            double an = axd.norm();

            mean_acc += an;
            ++num;
            if (an > max_acc)
            {
                max_acc = an;
            }
        }

        mean_acc = mean_acc / double(num);
        mean_a = mean_acc;
        max_a = max_acc;
    }
} // namespace ego_planner
