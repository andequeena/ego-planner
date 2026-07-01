#ifndef _GRADIENT_DESCENT_OPT_H_
#define _GRADIENT_DESCENT_OPT_H_

#include <iostream>
#include <vector>
#include <Eigen/Eigen>

using namespace std;

// 好像暂时没有使用

// 一个轻量级梯度下降优化器封装。
// 当前工程主要使用 lbfgs.hpp，这个类保留给简单目标函数或调试场景。
class GradientDescentOptimizer
{

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

  // 目标函数回调类型：
  // x 是当前变量，grad 由回调写入梯度，force_return 可要求优化器提前返回。
  typedef double (*objfunDef)(const Eigen::VectorXd &x, Eigen::VectorXd &grad, bool &force_return, void *data);
  enum RESULT
  {
    // 梯度足够小，认为找到局部极小值。
    FIND_MIN,
    // 参数非法或数值异常。
    FAILED,
    // 目标函数主动要求退出。
    RETURN_BY_ORDER,
    // 达到迭代或函数调用上限。
    REACH_MAX_ITERATION
  };

  GradientDescentOptimizer(int v_num, objfunDef objf, void *f_data)
  {
    // 保存变量维度、目标函数和用户上下文指针；优化器不拥有 f_data_ 生命周期。
    variable_num_ = v_num;
    objfun_ = objf;
    f_data_ = f_data;
  };

  // 设置最大迭代次数。
  void set_maxiter(int limit) { iter_limit_ = limit; }
  // 设置最大目标函数调用次数。
  void set_maxeval(int limit) { invoke_limit_ = limit; }
  // 相对变量变化阈值，当前实现中预留但未实际使用。
  void set_xtol_rel(double xtol_rel) { xtol_rel_ = xtol_rel; }
  // 绝对变量变化阈值，当前实现中预留但未实际使用。
  void set_xtol_abs(double xtol_abs) { xtol_abs_ = xtol_abs; }
  // 最小梯度范数阈值，小于该值时认为收敛。
  void set_min_grad(double min_grad) { min_grad_ = min_grad; }

  // 从 x_init_optimal 开始优化，返回时该变量保存最后的优化变量，opt_f 保存最后代价。
  RESULT optimize(Eigen::VectorXd &x_init_optimal, double &opt_f);

private:
  // 优化变量维度。
  int variable_num_{0};
  // 最大迭代次数。
  int iter_limit_{1e10};
  // 最大目标函数调用次数。
  int invoke_limit_{1e10};
  // 相对步长终止阈值，当前未使用。
  double xtol_rel_;
  // 绝对步长终止阈值，当前未使用。
  double xtol_abs_;
  // 梯度范数终止阈值。
  double min_grad_;
  // 时间上限预留字段，当前未使用。
  double time_limit_;
  // 传给目标函数的用户数据。
  void *f_data_;
  // 目标函数及梯度回调。
  objfunDef objfun_;
};

#endif
