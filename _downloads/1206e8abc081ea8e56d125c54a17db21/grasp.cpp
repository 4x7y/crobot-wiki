#include "crobot/kdl/grasp.hpp"

#include <numeric>

#include "crobot/common/assert.hpp"

using namespace crobot;
using namespace kdl;

//==============================================================================
Grasp::Grasp(ConstRigidBodyPtr body) : m_body(std::move(body)) {
  CROBOT_ASSERT_ERROR(m_body);
}

//==============================================================================
void Grasp::add_contact(BodyContact::Ptr contact) {
  m_contacts.push_back(contact);
}

//==============================================================================
void Grasp::add_contact(
    ContactModel contact_model,
    const math::Matrix4d& tform_obj,
    ConstRigidBodyPtr manipulator,
    const math::Matrix4d& tform_manip,
    double mu) {
  BodyContact::Ptr contact = std::make_shared<BodyContact>(
      contact_model,
      m_body,
      std::make_shared<BodyFrame>(m_body, tform_obj),
      manipulator,
      std::make_shared<BodyFrame>(manipulator, tform_manip),
      mu);
  m_contacts.push_back(contact);
}

//==============================================================================
void Grasp::add_contact(
    ContactModel contact_model, const math::Matrix4d& tform_obj, double mu) {
  BodyContact::Ptr contact = std::make_shared<BodyContact>(
      contact_model,
      m_body,
      std::make_shared<BodyFrame>(m_body, tform_obj),
      nullptr,
      nullptr,
      mu);
  m_contacts.push_back(contact);
}

//==============================================================================
void Grasp::add_contact(
    ContactModel contact_model,
    const math::Vector3d& bpos_obj,
    const math::Vector3d& normal_obj,
    double mu) {
  BodyContact::Ptr contact = std::make_shared<BodyContact>(
      contact_model,
      m_body,
      std::make_shared<BodyFrame>(m_body, bpos_obj, normal_obj),
      nullptr,
      nullptr,
      mu);
  m_contacts.push_back(contact);
}

//==============================================================================
void Grasp::delete_contact(const std::string& body_name) {
  m_contacts.erase(
      std::remove_if(
          m_contacts.begin(),
          m_contacts.end(),
          [&body_name](const BodyContact::Ptr& contact) {
            return contact->get_body2()->get_name() == body_name;
          }),
      m_contacts.end());
}

//==============================================================================
const std::vector<BodyContact::Ptr>& Grasp::get_contacts() const {
  return m_contacts;
}

//==============================================================================
BodyContact::Ptr Grasp::get_contact(const std::string& body_name) const {
  for (const auto& contact : m_contacts) {
    if (contact->get_body2()->get_name() == body_name) {
      return contact;
    }
  }

  CROBOT_ASSERT_ERROR(false, "Unknown contact body: %s", body_name.c_str());
  return nullptr;
}

//==============================================================================
bool Grasp::has_contact(const std::string& body_name) const {
  for (const auto& contact : m_contacts) {
    if (contact->get_body2()->get_name() == body_name) {
      return true;
    }
  }
  return false;
}

//==============================================================================
int Grasp::get_num_contacts() const {
  return static_cast<int>(m_contacts.size());
}

//==============================================================================
int Grasp::get_num_wrench_basis() const {
  return std::accumulate(
      m_contacts.begin(),
      m_contacts.end(),
      0,
      [](int sum, const auto& contact) {
        return sum + contact->get_num_wrench_basis();
      });
}

//==============================================================================
math::Matrix6X Grasp::get_grasp_matrix() const {
  math::Matrix6X G(6, get_num_wrench_basis());
  int col = 0;
  for (const auto& contact : m_contacts) {
    int m                = contact->get_num_wrench_basis();
    G.middleCols(col, m) = contact->get_contact_map();
    col += m;
  }
  return G;
}

//==============================================================================
ConstRigidBodyPtr Grasp::get_body() const {
  return m_body;
}
