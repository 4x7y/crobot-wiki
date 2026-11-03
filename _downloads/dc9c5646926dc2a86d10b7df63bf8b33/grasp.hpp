#pragma once

#include <memory>

#include "crobot/kdl/body_contact.hpp"
#include "crobot/kdl/rigid_body.hpp"

namespace crobot::kdl {

class Grasp {
public:
  using Ptr      = std::shared_ptr<Grasp>;
  using ConstPtr = std::shared_ptr<const Grasp>;

  virtual ~Grasp() = default;

  /**
   * @brief Construct a grasp with a body.
   *
   * @param body The RigidBody to grasp.
   */
  explicit Grasp(ConstRigidBodyPtr body);

  void add_contact(BodyContact::Ptr contact);

  /**
   * @brief Add a contact point to the grasp.
   *
   * @param contact_model The contact model
   * @param tform_manip The pose of the contact frame with respect to the
   * object frame
   * @param manipulator The manipulator that is grasping the object
   * @param tform_manip The pose of the contact frame with respect to the
   * manipulator's body frame
   * @param mu The friction coefficient
   */
  void add_contact(
      ContactModel contact_model,
      const math::Matrix4d& tform_obj,
      ConstRigidBodyPtr manipulator,
      const math::Matrix4d& tform_manip,
      double mu = 1.0);

  /**
   * @brief Add a contact point to the grasp without specifying the
   * end-effector.
   *
   * @param contact_model The contact model
   * @param tform_obj The pose of the contact frame with repsect to the
   * object frame
   * @param mu The friction coefficient
   */
  void add_contact(
      ContactModel contact_model,
      const math::Matrix4d& tform_obj,
      double mu = 1.0);

  /**
   * @brief Add a contact point to the grasp without specifying the
   * end-effector.
   *
   * @param contact_model The contact model
   * @param bpos_obj The position of the contact frame with repsect to the
   * object frame
   * @param normal_obj The normal vector of the contact frame in the object
   * frame. This is used to determine the orientation of the contact frame.
   * @param mu The friction coefficient
   */
  void add_contact(
      ContactModel contact_model,
      const math::Vector3d& bpos_obj,
      const math::Vector3d& normal_obj,
      double mu = 1.0);

  void delete_contact(const std::string& body_name);

  const std::vector<BodyContact::Ptr>& get_contacts() const;

  /**
   * @brief Get the contact on the object.
   *
   * @param body_name
   * @return ConstBodyContactPtr
   */
  BodyContact::Ptr get_contact(const std::string& body_name) const;

  bool has_contact(const std::string& body_name) const;

  /**
   * @brief Get the number of contacts in the grasp.
   *
   * @return int Number of contacts in the grasp
   */
  int get_num_contacts() const;

  /**
   * @brief Get the number of wrench basis
   *
   * @return int
   */
  int get_num_wrench_basis() const;

  /**
   * @brief Get the grasp matrix
   *
   * The grasp matrix G defines how forces exerted at the contact points are
   * mapped into wrenches.
   *
   */
  math::Matrix6X get_grasp_matrix() const;

  ConstRigidBodyPtr get_body() const;

private:
  ConstRigidBodyPtr m_body;
  std::vector<BodyContact::Ptr> m_contacts;
};

} // namespace crobot::kdl

