#include <bspline_opt/gradient_descent_optimizer.h>

#define RESET "\033[0m"
#define RED "\033[31m"

GradientDescentOptimizer::RESULT
GradientDescentOptimizer::optimize(Eigen::VectorXd &x_init_optimal, double &opt_f)
{
    // 基本参数检查：梯度阈值必须为正且足够大，否则无法可靠判断收敛。
    if (min_grad_ < 1e-10)
    {
        cout << RED << "min_grad_ is invalid:" << min_grad_ << RESET << endl;
        return FAILED;
    }
    // 至少需要初始点、第一次试探步和一次迭代更新，迭代上限过小没有优化意义。
    if (iter_limit_ <= 2)
    {
        cout << RED << "iter_limit_ is invalid:" << iter_limit_ << RESET << endl;
        return FAILED;
    }

    // f_data_ 是用户目标函数携带的上下文指针，优化器本身不解释其类型。
    void *f_data = f_data_;
    // 这里从 2 开始，是因为下面会先计算 x_k 和 x_kp1 两次目标函数调用。
    int iter = 2;
    int invoke_count = 2;
    // force_return 由目标函数写入，用于让外层规划器主动中断优化。
    bool force_return;
    // x_k / x_kp1 交替保存相邻两次迭代点，避免每轮额外拷贝。
    Eigen::VectorXd x_k(x_init_optimal), x_kp1(x_init_optimal.rows());
    double cost_k, cost_kp1, cost_min;
    Eigen::VectorXd grad_k(x_init_optimal.rows()), grad_kp1(x_init_optimal.rows());

    // 先在初始点计算一次代价和梯度，建立 Barzilai-Borwein 步长需要的第一组信息。
    cost_k = objfun_(x_k, grad_k, force_return, f_data);
    if (force_return)
        return RETURN_BY_ORDER;
    cost_min = cost_k;
    // 首次步长按最大梯度分量限幅，避免第一步把控制点移动过远。
    double max_grad = max(abs(grad_k.maxCoeff()), abs(grad_k.minCoeff()));
    constexpr double MAX_MOVEMENT_AT_FIRST_ITERATION = 0.1; // meter
    double alpha0 = max_grad < MAX_MOVEMENT_AT_FIRST_ITERATION ? 1.0 : (MAX_MOVEMENT_AT_FIRST_ITERATION / max_grad);
    x_kp1 = x_k - alpha0 * grad_k;
    // 计算第一步后的代价和梯度，后续即可使用相邻迭代差估计步长。
    cost_kp1 = objfun_(x_kp1, grad_kp1, force_return, f_data);
    if (force_return)
        return RETURN_BY_ORDER;
    if (cost_min > cost_kp1)
        cost_min = cost_kp1;

    /*** start iteration ***/
    while (++iter <= iter_limit_ && invoke_count <= invoke_limit_)
    {
        // s 是变量变化量，y 是梯度变化量；alpha = s^T y / y^T y 是 BB 步长。
        Eigen::VectorXd s = x_kp1 - x_k;
        Eigen::VectorXd y = grad_kp1 - grad_k;
        double alpha = s.dot(y) / y.dot(y);
        // y.dot(y) 过小或数值异常时，步长会失效，继续迭代可能造成发散。
        if (isnan(alpha) || isinf(alpha))
        {
            cout << RED << "step size invalid! alpha=" << alpha << RESET << endl;
            return FAILED;
        }

        if (iter % 2) // to aviod copying operations
        {
            do
            {
                // 奇数轮把新结果写回 x_k，和 x_kp1 交替作为“当前点/上一点”。
                x_k = x_kp1 - alpha * grad_kp1;
                cost_k = objfun_(x_k, grad_k, force_return, f_data);
                invoke_count++;
                if (force_return)
                    return RETURN_BY_ORDER;
                // 若不满足下降条件，则不断缩短步长，相当于回溯线搜索。
                alpha *= 0.5;
            } while (cost_k > cost_kp1 - 1e-4 * alpha * grad_kp1.transpose() * grad_kp1); // Armijo condition

            // 梯度范数足够小时认为已经到达局部极小点。
            if (grad_k.norm() < min_grad_)
            {
                opt_f = cost_k;
                return FIND_MIN;
            }
        }
        else
        {
            do
            {
                // 偶数轮把新结果写回 x_kp1，避免每轮交换 Eigen 向量。
                x_kp1 = x_k - alpha * grad_k;
                cost_kp1 = objfun_(x_kp1, grad_kp1, force_return, f_data);
                invoke_count++;
                if (force_return)
                    return RETURN_BY_ORDER;
                // Armijo 条件要求新代价比线性预测有足够下降，否则继续减半步长。
                alpha *= 0.5;
            } while (cost_kp1 > cost_k - 1e-4 * alpha * grad_k.transpose() * grad_k); // Armijo condition

            // 收敛判据同上，只是此轮最新梯度保存在 grad_kp1 中。
            if (grad_kp1.norm() < min_grad_)
            {
                opt_f = cost_kp1;
                return FIND_MIN;
            }
        }
    }

    // 退出循环说明达到迭代次数或函数调用次数上限，返回最后一次有效代价。
    opt_f = iter_limit_ % 2 ? cost_k : cost_kp1;
    return REACH_MAX_ITERATION;
}
