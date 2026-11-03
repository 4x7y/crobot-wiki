#include <memory>
#include <string>
#include <vector>

#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <yaml-cpp/yaml.h>

#include "crobot/ik/gik.hpp"
#include "crobot/ik/jacobian_ik.hpp"
#include "crobot/ik/jacobian_ik_target.hpp"
#include "crobot/ik/null_space_objective.hpp"
#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/matrix.hpp"

namespace py = pybind11;

void init_ik(py::module& m) {
  m.doc() = "Inverse Kinematics algorithms and solvers";

  // Enums
  py::enum_<crobot::ik::JacobianIKAlgorithm>(m, "JacobianIKAlgorithm")
      .value("TRANSPOSE", crobot::ik::JacobianIKAlgorithm::TRANSPOSE)
      .value("SVD", crobot::ik::JacobianIKAlgorithm::SVD)
      .value(
          "DAMPED_LEAST_SQUARE",
          crobot::ik::JacobianIKAlgorithm::DAMPED_LEAST_SQUARE);

  py::enum_<crobot::ik::GeneralizedIK::PotentialCost>(m, "PotentialCost")
      .value(
          "QPOS_QUADRATIC_POTENTIAL",
          crobot::ik::GeneralizedIK::PotentialCost::QPOS_QUADRATIC_POTENTIAL)
      .value(
          "QPOS_USHAPE_POTENTIAL",
          crobot::ik::GeneralizedIK::PotentialCost::QPOS_USHAPE_POTENTIAL)
      .value(
          "BODY_POSE_POTENTIAL",
          crobot::ik::GeneralizedIK::PotentialCost::BODY_POSE_POTENTIAL);

  // JacobianIK
  py::class_<crobot::ik::JacobianIK, std::shared_ptr<crobot::ik::JacobianIK>>(
      m, "JacobianIK")
      .def(
          py::init<
              crobot::kdl::ConstRigidBodyTreePtr,
              crobot::ik::JacobianIKAlgorithm,
              int,
              double>(),
          py::arg("rbtree"),
          py::arg("alg") = crobot::ik::JacobianIKAlgorithm::DAMPED_LEAST_SQUARE,
          py::arg("num_iters") = 100,
          py::arg("tol")       = 1e-5)
      .def(
          "add_target_selected_dofs",
          &crobot::ik::JacobianIK::add_target_selected_dofs)
      .def(
          "add_rolling_target_selected_dofs",
          &crobot::ik::JacobianIK::add_rolling_target_selected_dofs)
      .def(
          "fix_joint",
          py::overload_cast<const std::string&, const crobot::math::VectorXd&>(
              &crobot::ik::JacobianIK::fix_joint))
      .def(
          "fix_joint",
          py::overload_cast<const std::string&, double>(
              &crobot::ik::JacobianIK::fix_joint))
      .def(
          "fix_joint",
          py::overload_cast<const std::string&>(
              &crobot::ik::JacobianIK::fix_joint))
      .def("remove_fix_joint", &crobot::ik::JacobianIK::remove_fix_joint)
      .def("fix_body", &crobot::ik::JacobianIK::fix_body)
      .def("solve", &crobot::ik::JacobianIK::solve)
      .def("get_qpos", &crobot::ik::JacobianIK::get_qpos)
      .def("set_qpos", &crobot::ik::JacobianIK::set_qpos)
      .def(
          "set_null_space_objective",
          &crobot::ik::JacobianIK::set_null_space_objective);

  // GeneralizedIK
  py::class_<
      crobot::ik::GeneralizedIK,
      std::shared_ptr<crobot::ik::GeneralizedIK>>(m, "GeneralizedIK")
      .def(
          py::init<crobot::kdl::ConstRigidBodyTreePtr, bool>(),
          py::arg("rbtree"),
          py::arg("limited") = true)
      .def_static("LoadFromFile", &crobot::ik::GeneralizedIK::LoadFromFile)
      .def_static(
          "LoadFromYamlNode",
          py::overload_cast<
              crobot::kdl::ConstRigidBodyTreePtr,
              const YAML::Node&>(&crobot::ik::GeneralizedIK::LoadFromYamlNode))
      .def(
          "solve",
          &crobot::ik::GeneralizedIK::solve,
          py::arg("max_iter") = 1,
          py::arg("max_time") = 0.1,
          py::arg("verbose")  = false)
      .def("show_details", &crobot::ik::GeneralizedIK::show_details)
      .def(
          "add_position_target",
          py::overload_cast<
              const std::string&,
              const std::string&,
              const crobot::math::Vector3d&,
              const crobot::math::Vector3d&,
              double>(&crobot::ik::GeneralizedIK::add_position_target))
      .def(
          "add_position_target",
          py::overload_cast<
              const std::string&,
              const std::string&,
              const crobot::math::Vector3d&,
              double>(&crobot::ik::GeneralizedIK::add_position_target))
      .def(
          "add_bounding_box_constraint",
          &crobot::ik::GeneralizedIK::add_bounding_box_constraint)
      .def(
          "add_distance_target",
          py::overload_cast<
              const std::string&,
              const std::string&,
              const crobot::math::Vector3d&,
              const crobot::math::Vector3d&,
              double,
              double>(&crobot::ik::GeneralizedIK::add_distance_target))
      .def(
          "add_distance_target",
          py::overload_cast<
              const std::string&,
              const std::string&,
              const crobot::math::Vector3d&,
              double,
              double>(&crobot::ik::GeneralizedIK::add_distance_target))
      .def(
          "add_distance_target",
          py::overload_cast<
              const std::string&,
              const std::string&,
              const crobot::math::Vector3d&,
              const std::string&,
              const crobot::math::Vector3d&,
              double,
              double>(&crobot::ik::GeneralizedIK::add_distance_target))
      .def(
          "add_opposite_orientation_target",
          &crobot::ik::GeneralizedIK::add_opposite_orientation_target)
      .def(
          "add_cartesian_target",
          py::overload_cast<
              const std::string&,
              const std::string&,
              const crobot::math::Vector3d&,
              const crobot::math::Vector3d&,
              const crobot::math::Vector3d&>(
              &crobot::ik::GeneralizedIK::add_cartesian_target))
      .def(
          "add_cartesian_target",
          py::overload_cast<
              const std::string&,
              const std::string&,
              const crobot::math::Vector3d&,
              const crobot::math::Vector3d&>(
              &crobot::ik::GeneralizedIK::add_cartesian_target))
      .def(
          "add_orientation_target",
          &crobot::ik::GeneralizedIK::add_orientation_target)
      .def(
          "add_orientation_vector_target",
          &crobot::ik::GeneralizedIK::add_orientation_vector_target)
      .def(
          "add_aiming_target",
          py::overload_cast<
              const std::string&,
              const std::string&,
              const crobot::math::Vector3d&,
              const crobot::math::Vector3d&,
              const crobot::math::Vector3d&,
              double>(&crobot::ik::GeneralizedIK::add_aiming_target))
      .def(
          "add_aiming_target",
          py::overload_cast<
              const std::string&,
              const std::string&,
              const crobot::math::Vector3d&,
              const crobot::math::Vector3d&,
              double>(&crobot::ik::GeneralizedIK::add_aiming_target))
      .def(
          "add_aiming_target",
          py::overload_cast<
              const std::string&,
              const std::string&,
              const crobot::math::Vector3d&,
              double>(&crobot::ik::GeneralizedIK::add_aiming_target))
      .def(
          "add_joint_coupling_constraint",
          &crobot::ik::GeneralizedIK::add_joint_coupling_constraint)
      .def(
          "add_joint_position_constraint",
          &crobot::ik::GeneralizedIK::add_joint_position_constraint)
      .def(
          "fix_joint",
          static_cast<void (crobot::ik::GeneralizedIK::*)(
              const std::string&, double) const>(
              &crobot::ik::GeneralizedIK::fix_joint))
      .def("add_potential_cost", &crobot::ik::GeneralizedIK::add_potential_cost)
      .def("fix_joint_range", &crobot::ik::GeneralizedIK::fix_joint_range)
      .def(
          "validate_constraints",
          &crobot::ik::GeneralizedIK::validate_constraints)
      .def("set_init_qpos", &crobot::ik::GeneralizedIK::set_init_qpos)
      .def(
          "set_init_guess_limit",
          &crobot::ik::GeneralizedIK::set_init_guess_limit)
      .def("get_qpos", &crobot::ik::GeneralizedIK::get_qpos)
      .def("get_constriant", &crobot::ik::GeneralizedIK::get_constriant)
      .def(
          "add_contact_constraint",
          &crobot::ik::GeneralizedIK::add_contact_constraint)
      .def(
          "add_surface_point_contact_constraint",
          &crobot::ik::GeneralizedIK::add_surface_point_contact_constraint)
      .def(
          "add_object_contact_constraint",
          &crobot::ik::GeneralizedIK::add_object_contact_constraint);
}