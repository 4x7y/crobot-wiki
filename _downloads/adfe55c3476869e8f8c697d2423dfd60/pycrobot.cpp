#include <pybind11/eigen.h>
#include <pybind11/functional.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;

// Forward declarations of module initialization functions
void init_control(py::module&);
void init_kdl(py::module&);
void init_math(py::module&);
void init_optimization(py::module&);
void init_ik(py::module&);

PYBIND11_MODULE(pycrobot, m) {
  m.doc() = "Python bindings for CRobot library";

  // Create submodules
  py::module control_module = m.def_submodule("control", "Control algorithms");
  py::module kdl_module
      = m.def_submodule("kdl", "Kinematic and Dynamic Library");
  py::module math_module = m.def_submodule(
      "math", "Mathematical utilities and quaternion operations");
  py::module optimization_module
      = m.def_submodule("optimization", "Optimization algorithms and solvers");
  py::module ik_module
      = m.def_submodule("ik", "Inverse Kinematics algorithms and solvers");

  // Initialize submodules
  init_control(control_module);
  init_kdl(kdl_module);
  init_math(math_module);
  init_optimization(optimization_module);
  init_ik(ik_module);

  // Version information
  m.attr("__version__") = "0.1.0";
}
