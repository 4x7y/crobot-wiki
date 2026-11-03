// Include only the PID controller header for now
#include "crobot/control/pid.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/eigen.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>


namespace py = pybind11;
using namespace crobot;

void init_control(py::module &m) {
    // PID Controller
    py::class_<control::PIDController, std::shared_ptr<control::PIDController>>(m, "PIDController")
        .def(py::init<const math::VectorXd&, const math::VectorXd&, const math::VectorXd&,
                      std::function<math::VectorXd()>, std::function<math::VectorXd()>>(),
             py::arg("kp"), py::arg("ki"), py::arg("kd"), 
             py::arg("qpos_callback"), py::arg("qvel_callback"),
             "Create a PID controller with specified gains and state callbacks")
        .def("update", &control::PIDController::update, 
             py::arg("time"), py::arg("duration"),
             "Update controller state with current time and duration since last update")
        .def("set_desired", &control::PIDController::set_desired,
             py::arg("qpos"), py::arg("qvel"),
             "Set the desired position and velocity")
        .def("generate", &control::PIDController::generate,
             "Generate control output based on current state and desired state");
}
