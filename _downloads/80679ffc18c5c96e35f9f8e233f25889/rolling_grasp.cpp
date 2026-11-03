#include "crobot/kdl/rolling_grasp.hpp"

#include <numeric>

#include "crobot/common/assert.hpp"

using namespace crobot;
using namespace kdl;

//==============================================================================
RollingGrasp::RollingGrasp(ConstRigidBodyPtr body) : m_body(std::move(body)) {
  CROBOT_ASSERT_ERROR(m_body);
}

//==============================================================================
void RollingGrasp::add_contact(
    ContactModel contact_model,
    ConstRigidBodyPtr body1,
    math::SurfacePatch::ConstPtr surface1,
    math::Vector2d& contact_point1,
    ConstRigidBodyPtr body2,
    math::SurfacePatch::ConstPtr surface2,
    math::Vector2d& contact_point2,
    double mu) {
  RollingContact::Ptr contact = std::make_shared<RollingContact>(
      contact_model,
      body1,
      surface1,
      contact_point1,
      body2,
      surface2,
      contact_point2,
      mu);
  m_contacts.push_back(contact);
}

//==============================================================================
const std::vector<RollingContact::Ptr>& RollingGrasp::get_contacts() const {
  return m_contacts;
}

//==============================================================================
RollingContact::Ptr RollingGrasp::get_contact(
    const std::string& body_name) const {
  for (const auto& contact : m_contacts) {
    if (contact->get_body2()->get_name() == body_name) {
      return contact;
    }
  }

  CROBOT_ASSERT_ERROR(false, "Unknown contact body: %s", body_name.c_str());
  return nullptr;
}

//==============================================================================
int RollingGrasp::get_num_contacts() const {
  return static_cast<int>(m_contacts.size());
}

//==============================================================================
int RollingGrasp::get_num_wrench_basis() const {
  return std::accumulate(
      m_contacts.begin(),
      m_contacts.end(),
      0,
      [](int sum, const auto& contact) {
        return sum + contact->get_num_wrench_basis();
      });
}

//==============================================================================
math::Matrix6X RollingGrasp::get_grasp_matrix() const {
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
math::Matrix6X RollingGrasp::get_grasp_matrix_deriv() const {
  math::Matrix6X G_dot(6, get_num_wrench_basis());
  int col = 0;
  for (const auto& contact : m_contacts) {
    int m                    = contact->get_num_wrench_basis();
    G_dot.middleCols(col, m) = contact->get_contact_map_deriv();
    col += m;
  }
  return G_dot;
}

//==============================================================================
math::MatrixX6 RollingGrasp::get_grasp_matrix_pinv() const {
  math::MatrixX6 Ginv(get_num_wrench_basis(), 6);
  Ginv = get_grasp_matrix().completeOrthogonalDecomposition().pseudoInverse();
  return Ginv;
}

//==============================================================================
math::VectorXd RollingGrasp::get_ctc_xfrc_stack() const {
  math::VectorXd f(get_num_wrench_basis());
  int col = 0;
  for (const auto& contact : m_contacts) {
    // int m             = contact->get_num_wrench_basis();
    f.segment(col, 3) = contact->get_xfrc();
    col += 3;
  }
  return f;
}

//==============================================================================
math::VectorXd RollingGrasp::get_ctc_cfrc_stack(
    const std::vector<math::Matrix4d>& tforms) const {
  math::VectorXd f(get_num_wrench_basis());
  int col = 0;
  for (const auto& contact : m_contacts) {
    math::Vector3d xfrc = contact->get_xfrc();
    math::Matrix3d xmat = contact->get_xmat1(tforms);

    f.segment(col, 3) = xmat.transpose() * xfrc;
    col += 3;
  }
  return f;
}
