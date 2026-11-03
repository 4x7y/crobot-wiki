#pragma once

#include <memory>

#include "crobot/kdl/body_rolling_contact.hpp"
#include "crobot/kdl/grasp.hpp"
#include "crobot/kdl/rigid_body.hpp"

/**
 * this class is used to represent a grasp with rolling contacts
 * it is used in the real experiment (crobot/control/actions)
 * used together with BodyRollingContact
 * with patch switch
 */

namespace crobot::kdl {

class DynamicGrasp : public Grasp {
public:
  using Ptr      = std::shared_ptr<DynamicGrasp>;
  using ConstPtr = std::shared_ptr<const DynamicGrasp>;

  /**
   * @brief Construct a Dynamicgrasp with a body.
   *
   * @param body The RigidBody to Dynamicgrasp.
   */
  explicit DynamicGrasp(ConstRigidBodyPtr body);

  static std::shared_ptr<DynamicGrasp> from_yaml_node(
      const YAML::Node& node, ConstRigidBodyTreePtr rbtree);

  void add_contact(BodyRollingContact::Ptr contact);

  void add_contact(
      ContactModel contact_model,
      ConstRigidBodyPtr body1,
      math::SurfacePatchSwitch::Ptr surface1,
      math::Vector2d& contact_point1,
      ConstRigidBodyPtr body2,
      math::SurfacePatchSwitch::Ptr surface2,
      math::Vector2d& contact_point2,
      double mu = 1.0);

  /**
   * @brief Get the contact on the object.
   *
   * @param body_name
   * @return ConstBodyContactPtr
   */
  BodyRollingContact::Ptr get_dynamic_contact(
      const std::string& body_name) const;

  std::vector<BodyRollingContact::Ptr> get_dynamic_contacts() const;

  void delete_dynamic_contact(const std::string& body_name);

  DynamicGrasp::Ptr clone() const;

  using Grasp::add_contact;

  // get contact force stack in the world frame
  math::VectorXd get_ctc_xfrc_stack() const;

  // get contact force stack in the contact frame
  math::VectorXd get_ctc_cfrc_stack(
      const std::vector<math::Matrix4d>& tforms) const;

  // save grasp
  YAML::Node to_yaml_node() const;

private:
  std::vector<BodyRollingContact::Ptr> m_dynamic_contacts;
};

} // namespace crobot::kdl

