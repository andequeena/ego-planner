#include <iostream>
#include <traj_utils/polynomial_traj.h>

PolynomialTraj PolynomialTraj::minSnapTraj(const Eigen::MatrixXd &Pos, const Eigen::Vector3d &start_vel,
                                           const Eigen::Vector3d &end_vel, const Eigen::Vector3d &start_acc,
                                           const Eigen::Vector3d &end_acc, const Eigen::VectorXd &Time)
{
    // 生成一条经过所有给定位置点 Pos 的分段五次多项式轨迹。
    // 优化目标是 minimum snap，这里的 snap 指位置对时间的四阶导数；
    // snap 越小，轨迹通常越平顺，适合无人机这类需要平滑高阶导数的系统。
    // Pos 的每一列是一个路径点，因此 seg_num = Time.size() 表示多项式段数，
    // 若有 m 段轨迹，就需要 m + 1 个位置点。
    int seg_num = Time.size();
    // 每一段在 x/y/z 三个方向上各用一个五次多项式：
    // p(t)=c0+c1*t+c2*t^2+c3*t^3+c4*t^4+c5*t^5，共 6 个系数。
    // poly_coeff 每行保存一段轨迹的 18 个系数：x 的 6 个、y 的 6 个、z 的 6 个。
    Eigen::MatrixXd poly_coeff(seg_num, 3 * 6);
    // Px/Py/Pz 是所有段在对应坐标轴上的多项式系数，按“每段 6 个系数”串起来。
    Eigen::VectorXd Px(6 * seg_num), Py(6 * seg_num), Pz(6 * seg_num);

    int num_f, num_p; // number of fixed and free variables
    int num_d;        // number of all segments' derivatives

    // 计算阶乘，用在多项式求导的系数上。
    // 例如 t^5 的三阶导是 5*4*3*t^2，对应 Factorial(5)/Factorial(2)。
    const static auto Factorial = [](int x)
    {
        int fac = 1;
        for (int i = x; i > 0; i--)
            fac = fac * i;
        return fac;
    };

    /* ---------- end point derivative ---------- */
    // D* 不是多项式系数，而是“每段首尾端点的导数约束”。
    // 对第 k 段，6 个量依次表示：
    // [起点位置, 终点位置, 起点速度, 终点速度, 起点加速度, 终点加速度]。
    // x/y/z 三个方向分别构造一份，后面再通过矩阵映射求多项式系数。
    Eigen::VectorXd Dx = Eigen::VectorXd::Zero(seg_num * 6);
    Eigen::VectorXd Dy = Eigen::VectorXd::Zero(seg_num * 6);
    Eigen::VectorXd Dz = Eigen::VectorXd::Zero(seg_num * 6);

    for (int k = 0; k < seg_num; k++)
    {
        /* position to derivative */
        // 每段的起点、终点位置是固定的，直接来自相邻路径点 Pos.col(k) 和 Pos.col(k+1)。
        Dx(k * 6) = Pos(0, k);
        Dx(k * 6 + 1) = Pos(0, k + 1);
        Dy(k * 6) = Pos(1, k);
        Dy(k * 6 + 1) = Pos(1, k + 1);
        Dz(k * 6) = Pos(2, k);
        Dz(k * 6 + 1) = Pos(2, k + 1);

        if (k == 0)
        {
            // 第一段的起点速度、起点加速度由外部给定，用来满足当前机器人状态。
            Dx(k * 6 + 2) = start_vel(0);
            Dy(k * 6 + 2) = start_vel(1);
            Dz(k * 6 + 2) = start_vel(2);

            Dx(k * 6 + 4) = start_acc(0);
            Dy(k * 6 + 4) = start_acc(1);
            Dz(k * 6 + 4) = start_acc(2);
        }
        else if (k == seg_num - 1)
        {
            // 最后一段的终点速度、终点加速度由外部给定，用来约束轨迹末端状态。
            Dx(k * 6 + 3) = end_vel(0);
            Dy(k * 6 + 3) = end_vel(1);
            Dz(k * 6 + 3) = end_vel(2);

            Dx(k * 6 + 5) = end_acc(0);
            Dy(k * 6 + 5) = end_acc(1);
            Dz(k * 6 + 5) = end_acc(2);
        }
    }

    /* ---------- Mapping Matrix A ---------- */
    // A 是从“多项式系数 P”到“端点导数 D”的映射矩阵：D = A * P。
    // 每一段都有一个 6x6 的 Ab，整条轨迹的 A 是把各段 Ab 放在对角线上的块对角矩阵。
    Eigen::MatrixXd Ab;
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

    for (int k = 0; k < seg_num; k++)
    {
        Ab = Eigen::MatrixXd::Zero(6, 6);
        for (int i = 0; i < 3; i++)
        {
            // 偶数行表示 t=0 处的 0/1/2 阶导数。
            // 由于 t=0，高次项都消失，只剩对应阶数的系数乘阶乘。
            Ab(2 * i, i) = Factorial(i);
            // 奇数行表示 t=Time(k) 处的 0/1/2 阶导数。
            // j 从 i 开始，因为低于 i 次的项求 i 阶导后为 0。
            for (int j = i; j < 6; j++)
                Ab(2 * i + 1, j) = Factorial(j) / Factorial(j - i) * pow(Time(k), j - i);
        }
        A.block(k * 6, k * 6, 6, 6) = Ab;
    }

    /* ---------- Produce Selection Matrix C' ---------- */
    // Ct 是选择/重排矩阵，用来把每段的端点导数 D 按“固定变量 Df + 自由变量 Dp”的形式组织起来。
    // 固定变量包括：所有 waypoint 位置、整条轨迹起终点速度、起终点加速度。
    // 自由变量包括：中间 waypoint 处的速度和加速度，minimum snap 会求出它们的最优值。
    Eigen::MatrixXd Ct, C;

    num_f = 2 * seg_num + 4; // 3 + 3 + (seg_num - 1) * 2 = 2m + 4
    num_p = 2 * seg_num - 2; //(seg_num - 1) * 2 = 2m - 2
    num_d = 6 * seg_num;
    Ct = Eigen::MatrixXd::Zero(num_d, num_f + num_p);
    // 起点相关约束：起点位置、起点速度、起点加速度，以及第一段终点处的自由导数入口。
    Ct(0, 0) = 1;
    Ct(2, 1) = 1;
    Ct(4, 2) = 1; // stack the start point
    Ct(1, 3) = 1;
    Ct(3, 2 * seg_num + 4) = 1;
    Ct(5, 2 * seg_num + 5) = 1;

    // 终点相关约束：最后一段起点位置、终点位置、终点速度、终点加速度等。
    // 注意这里的索引是在把所有段的导数变量串成一个长向量后定位。
    Ct(6 * (seg_num - 1) + 0, 2 * seg_num + 0) = 1;
    Ct(6 * (seg_num - 1) + 1, 2 * seg_num + 1) = 1; // Stack the end point
    Ct(6 * (seg_num - 1) + 2, 4 * seg_num + 0) = 1;
    Ct(6 * (seg_num - 1) + 3, 2 * seg_num + 2) = 1; // Stack the end point
    Ct(6 * (seg_num - 1) + 4, 4 * seg_num + 1) = 1;
    Ct(6 * (seg_num - 1) + 5, 2 * seg_num + 3) = 1; // Stack the end point

    for (int j = 2; j < seg_num; j++)
    {
        // 中间段的公共连接点要在相邻两段中共享同一个位置；
        // 同时中间点的速度、加速度作为自由变量，由闭式解自动选择。
        Ct(6 * (j - 1) + 0, 2 + 2 * (j - 1) + 0) = 1;
        Ct(6 * (j - 1) + 1, 2 + 2 * (j - 1) + 1) = 1;
        Ct(6 * (j - 1) + 2, 2 * seg_num + 4 + 2 * (j - 2) + 0) = 1;
        Ct(6 * (j - 1) + 3, 2 * seg_num + 4 + 2 * (j - 1) + 0) = 1;
        Ct(6 * (j - 1) + 4, 2 * seg_num + 4 + 2 * (j - 2) + 1) = 1;
        Ct(6 * (j - 1) + 5, 2 * seg_num + 4 + 2 * (j - 1) + 1) = 1;
    }

    C = Ct.transpose();

    // Dx1/Dy1/Dz1 是重排后的导数变量向量：
    // 前 num_f 个是固定变量 Df，后 num_p 个是待求的自由变量 Dp。
    Eigen::VectorXd Dx1 = C * Dx;
    Eigen::VectorXd Dy1 = C * Dy;
    Eigen::VectorXd Dz1 = C * Dz;

    /* ---------- minimum snap matrix ---------- */
    // Q 是 snap 积分代价矩阵，使得单轴代价可以写成 P^T * Q * P。
    // 五次多项式的 snap 从四阶导开始；这份代码实际从 i,j=3 开始构造，
    // 对应的是三阶导 jerk 的积分形式，变量命名沿用了 minimum snap 的说法。
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(seg_num * 6, seg_num * 6);

    for (int k = 0; k < seg_num; k++)
    {
        for (int i = 3; i < 6; i++)
        {
            for (int j = 3; j < 6; j++)
            {
                Q(k * 6 + i, k * 6 + j) =
                    i * (i - 1) * (i - 2) * j * (j - 1) * (j - 2) / (i + j - 5) * pow(Time(k), (i + j - 5));
            }
        }
    }

    /* ---------- R matrix ---------- */
    // 因为 D = A * P，所以 P = A^{-1} * D。
    // 又因为 D = Ct * [Df; Dp]，代入 P^T Q P 后得到 R：
    // cost = [Df; Dp]^T * R * [Df; Dp]。
    Eigen::MatrixXd R = C * A.transpose().inverse() * Q * A.inverse() * Ct;

    // 取出固定部分 Df。三轴共享同一个 R，只是固定导数数值不同。
    Eigen::VectorXd Dxf(2 * seg_num + 4), Dyf(2 * seg_num + 4), Dzf(2 * seg_num + 4);

    Dxf = Dx1.segment(0, 2 * seg_num + 4);
    Dyf = Dy1.segment(0, 2 * seg_num + 4);
    Dzf = Dz1.segment(0, 2 * seg_num + 4);

    Eigen::MatrixXd Rff(2 * seg_num + 4, 2 * seg_num + 4);
    Eigen::MatrixXd Rfp(2 * seg_num + 4, 2 * seg_num - 2);
    Eigen::MatrixXd Rpf(2 * seg_num - 2, 2 * seg_num + 4);
    Eigen::MatrixXd Rpp(2 * seg_num - 2, 2 * seg_num - 2);

    // 把 R 按固定变量 Df 和自由变量 Dp 分块：
    // [Rff Rfp; Rpf Rpp]。
    Rff = R.block(0, 0, 2 * seg_num + 4, 2 * seg_num + 4);
    Rfp = R.block(0, 2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2);
    Rpf = R.block(2 * seg_num + 4, 0, 2 * seg_num - 2, 2 * seg_num + 4);
    Rpp = R.block(2 * seg_num + 4, 2 * seg_num + 4, 2 * seg_num - 2, 2 * seg_num - 2);

    /* ---------- close form solution ---------- */

    // 对自由变量 Dp 求导并令导数为 0，可得到闭式解：
    // Dp = -Rpp^{-1} * Rfp^T * Df。
    // 这一步求出的就是所有中间点的最优速度、加速度。
    Eigen::VectorXd Dxp(2 * seg_num - 2), Dyp(2 * seg_num - 2), Dzp(2 * seg_num - 2);
    Dxp = -(Rpp.inverse() * Rfp.transpose()) * Dxf;
    Dyp = -(Rpp.inverse() * Rfp.transpose()) * Dyf;
    Dzp = -(Rpp.inverse() * Rfp.transpose()) * Dzf;

    // 把求出的自由导数填回完整导数向量。
    Dx1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dxp;
    Dy1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dyp;
    Dz1.segment(2 * seg_num + 4, 2 * seg_num - 2) = Dzp;

    // 由完整导数约束反解出各段多项式系数。
    // 此处系数顺序是低次到高次：[c0, c1, ..., c5]。
    Px = (A.inverse() * Ct) * Dx1;
    Py = (A.inverse() * Ct) * Dy1;
    Pz = (A.inverse() * Ct) * Dz1;

    for (int i = 0; i < seg_num; i++)
    {
        poly_coeff.block(i, 0, 1, 6) = Px.segment(i * 6, 6).transpose();
        poly_coeff.block(i, 6, 1, 6) = Py.segment(i * 6, 6).transpose();
        poly_coeff.block(i, 12, 1, 6) = Pz.segment(i * 6, 6).transpose();
    }

    /* ---------- use polynomials ---------- */
    PolynomialTraj poly_traj;
    for (int i = 0; i < poly_coeff.rows(); ++i)
    {
        // 先取出第 i 段三个轴的 6 个系数。
        vector<double> cx(6), cy(6), cz(6);
        for (int j = 0; j < 6; ++j)
        {
            cx[j] = poly_coeff(i, j), cy[j] = poly_coeff(i, j + 6), cz[j] = poly_coeff(i, j + 12);
        }
        // PolynomialTraj::evaluate() 里按“高次到低次”的顺序做点乘，
        // 所以这里要把 [c0,c1,...,c5] 反转成 [c5,c4,...,c0] 后存入轨迹对象。
        reverse(cx.begin(), cx.end());
        reverse(cy.begin(), cy.end());
        reverse(cz.begin(), cz.end());
        double ts = Time(i);
        poly_traj.addSegment(cx, cy, cz, ts);
    }

    return poly_traj;
}

PolynomialTraj PolynomialTraj::one_segment_traj_gen(const Eigen::Vector3d &start_pt, const Eigen::Vector3d &start_vel, const Eigen::Vector3d &start_acc,
                                                    const Eigen::Vector3d &end_pt, const Eigen::Vector3d &end_vel, const Eigen::Vector3d &end_acc,
                                                    double t)
{
    // 生成单段五次多项式轨迹。
    // 给定起点/终点的位置、速度、加速度，共 6 个边界条件，正好解出 6 个多项式系数。
    // 这里的系数顺序是高次到低次：[a5,a4,a3,a2,a1,a0]，
    // 对应 p(t)=a5*t^5+a4*t^4+a3*t^3+a2*t^2+a1*t+a0。
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(6, 6), Crow(1, 6);
    Eigen::VectorXd Bx(6), By(6), Bz(6);

    // 配置矩阵
    // [0 0 0 0 0 1]
    // [0 0 0 0 1 0]
    // [0 0 0 2 0 0]
    // [T ^ 5 T ^ 4 T ^ 3 T ^ 2 T 1]
    // [5T ^ 4 4T ^ 3 3T ^ 2 2T 1 0]
    // [20T ^ 3 12T ^ 2 6T 2 0 0]

    // 前三行约束 t=0 时的位置、速度、加速度。
    // 因为 t=0 时只有常数项、一次项、二次项会留下，所以矩阵很稀疏。
    C(0, 5) = 1;
    C(1, 4) = 1;
    C(2, 3) = 2;
    // 后三行约束 t=T 时的位置、速度、加速度。
    Crow << pow(t, 5), pow(t, 4), pow(t, 3), pow(t, 2), t, 1;
    C.row(3) = Crow;
    Crow << 5 * pow(t, 4), 4 * pow(t, 3), 3 * pow(t, 2), 2 * t, 1, 0;
    C.row(4) = Crow;
    Crow << 20 * pow(t, 3), 12 * pow(t, 2), 6 * t, 2, 0, 0;
    C.row(5) = Crow;

    Bx << start_pt(0), start_vel(0), start_acc(0), end_pt(0), end_vel(0), end_acc(0);
    By << start_pt(1), start_vel(1), start_acc(1), end_pt(1), end_vel(1), end_acc(1);
    Bz << start_pt(2), start_vel(2), start_acc(2), end_pt(2), end_vel(2), end_acc(2);

    // 分别求解 x/y/z 三个方向的线性方程 C * coef = boundary。
    // colPivHouseholderQr() 是 Eigen 的 QR 分解求解器，比直接 inverse() 数值上更稳一些。
    Eigen::VectorXd Cofx = C.colPivHouseholderQr().solve(Bx);
    Eigen::VectorXd Cofy = C.colPivHouseholderQr().solve(By);
    Eigen::VectorXd Cofz = C.colPivHouseholderQr().solve(Bz);

    // addSegment 期望的系数顺序同样是高次到低次，这里 Cof* 本来就是这个顺序，所以不用 reverse。
    vector<double> cx(6), cy(6), cz(6);
    for (int i = 0; i < 6; i++)
    {
        cx[i] = Cofx(i);
        cy[i] = Cofy(i);
        cz[i] = Cofz(i);
    }

    PolynomialTraj poly_traj;
    poly_traj.addSegment(cx, cy, cz, t);

    return poly_traj;
}
