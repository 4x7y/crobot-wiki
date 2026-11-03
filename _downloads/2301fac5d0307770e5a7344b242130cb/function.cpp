#include "crobot/optimization/function.hpp"

#include "crobot/common/logging.hpp"

using namespace crobot;
using namespace optimization;

//==============================================================================
Function::Function(const std::string& _name) : mName(_name) {
  // Do nothing
}

//==============================================================================
Function::~Function() {
  // Do nothing
}

//==============================================================================
void Function::setName(const std::string& _newName) {
  mName = _newName;
}

//==============================================================================
const std::string& Function::getName() const {
  return mName;
}

//==============================================================================
void Function::evalGradient(
    const Eigen::VectorXd& /*_x*/, Eigen::Map<Eigen::VectorXd> /*_grad*/) {
  CROBOT_LOG_WARN(
      "Gradient is not provided by function named [{}]. Use gradient-free "
      "algorithm.",
      mName);
}

//==============================================================================
void Function::evalGradient(const Eigen::VectorXd& _x, Eigen::VectorXd& _grad) {
  Eigen::Map<Eigen::VectorXd> tmpGrad(_grad.data(), _grad.size());
  evalGradient(_x, tmpGrad);
}

//==============================================================================
void Function::evalHessian(
    const Eigen::VectorXd& /*_x*/,
    Eigen::Map<Eigen::VectorXd, Eigen::RowMajor> /*_Hess*/) {
  CROBOT_LOG_WARN(
      "Hessian is not provided by funciton named [{}]. Use Hessian-free "
      "algorithm.",
      mName);
}

//==============================================================================
ModularFunction::ModularFunction(const std::string& _name) : Function(_name) {
  clearCostFunction();
  clearGradientFunction();
  clearHessianFunction();
}

//==============================================================================
ModularFunction::~ModularFunction() {
  // Do nothing
}

//==============================================================================
double ModularFunction::eval(const Eigen::VectorXd& _x) {
  return mCostFunction(_x);
}

//==============================================================================
void ModularFunction::evalGradient(
    const Eigen::VectorXd& _x, Eigen::Map<Eigen::VectorXd> _grad) {
  mGradientFunction(_x, _grad);
}

//==============================================================================
void ModularFunction::evalHessian(
    const Eigen::VectorXd& _x,
    Eigen::Map<Eigen::VectorXd, Eigen::RowMajor> _Hess) {
  mHessianFunction(_x, _Hess);
}

//==============================================================================
void ModularFunction::setCostFunction(CostFunction _cost) {
  mCostFunction = _cost;
}

//==============================================================================
void ModularFunction::clearCostFunction(bool _printWarning) {
  mCostFunction = [=](const Eigen::VectorXd&) {
    if (_printWarning) {
      CROBOT_LOG_ERROR(
          "A cost function has not yet been assigned to the ModularFunction "
          "named [{}]. Returning 0.0",
          mName);
    }
    return 0;
  };
}

//==============================================================================
void ModularFunction::setGradientFunction(GradientFunction _gradient) {
  mGradientFunction = _gradient;
}

//==============================================================================
void ModularFunction::clearGradientFunction() {
  mGradientFunction
      = [&](const Eigen::VectorXd& _x, Eigen::Map<Eigen::VectorXd> _grad) {
          this->Function::evalGradient(_x, _grad);
        };
}

//==============================================================================
void ModularFunction::setHessianFunction(HessianFunction _hessian) {
  mHessianFunction = _hessian;
}

//==============================================================================
void ModularFunction::clearHessianFunction() {
  mHessianFunction = [&](const Eigen::VectorXd& _x,
                         Eigen::Map<Eigen::VectorXd, Eigen::RowMajor> _Hess) {
    this->Function::evalHessian(_x, _Hess);
  };
}

//==============================================================================
NullFunction::NullFunction(const std::string& _name) : Function(_name) {
  // Do nothing
}

//==============================================================================
NullFunction::~NullFunction() {
  // Do nothing
}

//==============================================================================
double NullFunction::eval(const Eigen::VectorXd&) {
  return 0;
}

//==============================================================================
void NullFunction::evalGradient(
    const Eigen::VectorXd& _x, Eigen::Map<Eigen::VectorXd> _grad) {
  _grad.resize(_x.size());
  _grad.setZero();
}

//==============================================================================
void NullFunction::evalHessian(
    const Eigen::VectorXd& _x,
    Eigen::Map<Eigen::VectorXd, Eigen::RowMajor> _Hess) {
  _Hess.resize(pow(_x.size(), 2));
  _Hess.setZero();
}

//==============================================================================
MultiFunction::MultiFunction() {
  // Do nothing
}

//==============================================================================
MultiFunction::~MultiFunction() {
  // Do nothing
}
