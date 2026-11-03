#include "crobot/kdl/dynamic_grasp.hpp"

#include <numeric>

#include "crobot/common/assert.hpp"

using namespace crobot;
using namespace kdl;

//==============================================================================
DynamicGrasp::DynamicGrasp(ConstRigidBodyPtr body) : Grasp(std::move(body)) {
}

//==============================================================================
std::shared_ptr<DynamicGrasp> DynamicGrasp::from_yaml_node(
    const YAML::Node& node, ConstRigidBodyTreePtr rbtree) {
  CROBOT_ASSERT_ERROR(node["type"].as<std::string>() == "DynamicGrasp");
  ConstRigidBodyPtr body  = rbtree->get_body(node["body"].as<std::string>());
  DynamicGrasp::Ptr grasp = std::make_shared<DynamicGrasp>(body);

  for (const auto& contact_node : node["contacts"]) {
    grasp->add_contact(
        BodyRollingContact::from_yaml_node(contact_node, rbtree));
  }
  return grasp;
}

//==============================================================================
void DynamicGrasp::add_contact(BodyRollingContact::Ptr contact) {
  m_dynamic_contacts.push_back(contact);
  Grasp::add_contact(contact);
}

//==============================================================================
void DynamicGrasp::add_contact(
    ContactModel contact_model,
    ConstRigidBodyPtr body1,
    math::SurfacePatchSwitch::Ptr surface1,
    math::Vector2d& contact_point1,
    ConstRigidBodyPtr body2,
    math::SurfacePatchSwitch::Ptr surface2,
    math::Vector2d& contact_point2,
    double mu) {
  BodyRollingContact::Ptr contact = std::make_shared<BodyRollingContact>(
      contact_model,
      body1,
      surface1->clone(),
      contact_point1,
      body2,
      surface2->clone(),
      contact_point2,
      mu);
  m_dynamic_contacts.push_back(contact);
  Grasp::add_contact(contact);
}

//==============================================================================
BodyRollingContact::Ptr DynamicGrasp::get_dynamic_contact(
    const std::string& body_name) const {
  for (auto& contact : m_dynamic_contacts) {
    if (contact->get_body2()->get_name() == body_name)
      return contact;
  }
  CROBOT_ASSERT_ERROR(
      false, "Unknown dynamic contact body: %s", body_name.c_str());
  return nullptr;
}

//==============================================================================
std::vector<BodyRollingContact::Ptr> DynamicGrasp::get_dynamic_contacts()
    const {
  return m_dynamic_contacts;
}

//==============================================================================
void DynamicGrasp::delete_dynamic_contact(const std::string& body_name) {
  for (auto it = m_dynamic_contacts.begin(); it != m_dynamic_contacts.end();) {
    if ((*it)->get_body2()->get_name() == body_name) {
      it = m_dynamic_contacts.erase(it);
    }
    else {
      ++it;
    }
  }
  delete_contact(body_name);
}

//==============================================================================
math::VectorXd DynamicGrasp::get_ctc_xfrc_stack() const {
  math::VectorXd f(get_num_wrench_basis());
  int col = 0;
  for (const auto& contact : m_dynamic_contacts) {
    // int m             = contact->get_num_wrench_basis();
    f.segment(col, 3) = contact->get_xfrc();
    col += 3;
  }
  return f;
}

//==============================================================================
math::VectorXd DynamicGrasp::get_ctc_cfrc_stack(
    const std::vector<math::Matrix4d>& tforms) const {
  math::VectorXd f(get_num_wrench_basis());
  int col = 0;
  for (const auto& contact : m_dynamic_contacts) {
    math::Vector3d xfrc = contact->get_xfrc();
    math::Matrix3d xmat = contact->get_xmat1(tforms);

    f.segment(col, 3) = xmat.transpose() * xfrc;
    col += 3;
  }
  return f;
}

//==============================================================================
DynamicGrasp::Ptr DynamicGrasp::clone() const {
  DynamicGrasp::Ptr grasp = std::make_shared<DynamicGrasp>(get_body());
  for (auto& contact : m_dynamic_contacts) {
    grasp->add_contact(contact->clone());
  }
  return grasp;
}

//==============================================================================
YAML::Node DynamicGrasp::to_yaml_node() const {
  YAML::Node node;
  node["type"] = "DynamicGrasp";
  node["body"] = get_body()->get_name();
  for (const auto& contact : m_dynamic_contacts) {
    node["contacts"].push_back(contact->to_yaml_node());
  }
  return node;
}
