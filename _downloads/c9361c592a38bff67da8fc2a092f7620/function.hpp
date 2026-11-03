#pragma once

#include <functional>
#include <memory>
#include <vector>

#include "crobot/math/math.hpp"

namespace crobot::optimization {

class Function {
public:
  /// Constructor
  explicit Function(const std::string& _name = "function");

  /// Destructor
  virtual ~Function();

  /// Sets the name of this Function
  virtual void setName(const std::string& newName);

  /// Returns the name of this Function
  const std::string& getName() const;

  /// Evaluates and returns the objective function at the point x
  virtual double eval(const math::VectorXd& x) = 0;

  /// Evaluates and returns the objective function at the point x
  virtual void evalGradient(
      const math::VectorXd& _x, math::MapToVectorXd _grad);

  /// Evaluates and return the objective function at the point x.
  ///
  /// If you have a raw array that the gradient will be passed in, then use
  /// evalGradient(const math::VectorXd&, math::MapToVectorXd)
  /// for better performance.
  void evalGradient(const math::VectorXd& _x, math::VectorXd& _grad);

  /// Evaluates and return the objective function at the point x
  virtual void evalHessian(
      const math::VectorXd& _x,
      Eigen::Map<math::VectorXd, Eigen::RowMajor> _Hess);

protected:
  /// Name of this function
  std::string mName;
};

typedef std::shared_ptr<Function> FunctionPtr;
typedef std::unique_ptr<Function> UniqueFunctionPtr;

typedef std::function<double(const math::VectorXd&)> CostFunction;

typedef std::function<void(const math::VectorXd&, math::MapToVectorXd)>
    GradientFunction;

typedef std::function<void(
    const math::VectorXd&, Eigen::Map<math::VectorXd, Eigen::RowMajor>)>
    HessianFunction;

/// ModularFunction uses C++11 std::function to allow you to easily swap
/// out the cost function, gradient function, and Hessian function during
/// runtime for an optimization::Function instance.
class ModularFunction : public Function {
public:
  /// Constructor
  explicit ModularFunction(const std::string& _name = "modular_function");

  /// Destructor
  ~ModularFunction() override;

  /// eval() will now call whatever CostFunction you set using
  /// setCostFunction()
  double eval(const math::VectorXd& _x) override;

  /// evalGradient() will now call whatever GradientFunction you set
  /// using setGradientFunction()
  void evalGradient(
      const math::VectorXd& _x, math::MapToVectorXd _grad) override;

  /// evalHessian() will now call whatever HessianFunction you set using
  /// setHessianFunction()
  void evalHessian(
      const math::VectorXd& _x,
      Eigen::Map<math::VectorXd, Eigen::RowMajor> _Hess) override;

  /// Set the function that gets called by eval()
  void setCostFunction(CostFunction _cost);

  /// Replace the cost function with a constant-zero function. Passing in
  /// true will cause a warning to be printed out whenever eval() is called.
  void clearCostFunction(bool _printWarning = true);

  /// Set the function that gets called by evalGradient()
  void setGradientFunction(GradientFunction _gradient);

  /// Replace the gradient function with the default evalGradient() of
  /// the base Function class. A warning will be printed whenever evalGradient()
  /// gets called.
  void clearGradientFunction();

  /// Set the function that gets called by evalHessian()
  void setHessianFunction(HessianFunction _hessian);

  /// Replace the Hessian function with the default evalHessian() of the
  /// base Function class. A warning will be printed whenever evalHessian() gets
  /// called.
  void clearHessianFunction();

protected:
  /// Storage for the cost function
  CostFunction mCostFunction;

  /// Storage for the gradient function
  GradientFunction mGradientFunction;

  /// Storage for the Hessian function
  HessianFunction mHessianFunction;
};

/// NullFunction is a constant-zero Function
class NullFunction : public Function {
public:
  /// Constructor
  explicit NullFunction(const std::string& _name = "null_function");

  /// Destructor
  ~NullFunction() override;

  /// eval() will always return exactly zero
  double eval(const math::VectorXd&) override;

  /// evalGradient will always set _grad to a zero vector that
  /// matches the dimensionality of _x
  void evalGradient(
      const math::VectorXd& _x, math::MapToVectorXd _grad) override;

  /// evalHessian() will always set _Hess to a zero matrix that matches
  /// the dimensionality of _x
  void evalHessian(
      const math::VectorXd& _x,
      Eigen::Map<math::VectorXd, Eigen::RowMajor> _Hess) override;
};

/// class MultiFunction
class MultiFunction {
public:
  /// Constructor
  MultiFunction();

  /// Destructor
  virtual ~MultiFunction();

  /// Operator ()
  virtual void operator()(
      const math::VectorXd& _x,
      math::MapToVectorXd& _f,
      Eigen::Map<Eigen::MatrixXd>& _grad)
      = 0;
};

} // namespace crobot::optimization

