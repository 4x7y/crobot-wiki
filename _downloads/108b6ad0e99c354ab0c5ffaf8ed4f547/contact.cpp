#include "crobot/kdl/contact.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/common/logging.hpp"

using namespace crobot;
using namespace kdl;

//==============================================================================
Contact::Contact(ContactModel contact_model, double mu)
  : m_model(contact_model), m_mu(mu) {
}

//==============================================================================
double Contact::get_mu() const {
  return m_mu;
}

//==============================================================================
int Contact::get_num_wrench_basis() const {
  switch (m_model) {
    case FRICTIONLESS_POINT_CONTACT:
      return 1;
    case POINT_CONTACT_WITH_FRICTION:
      return 3;
    case SOFT_FINGER_CONTACT:
      return 4;
    default:
      CROBOT_LOG_ERROR("Unknown contact model.");
      return 0;
  }
}

//==============================================================================
math::MatrixX6 Contact::get_wrench_basis() const {
  return get_selection_matrix().transpose();
}

//==============================================================================
math::MatrixX6 Contact::get_selection_matrix() const {
  math::MatrixX6 S;
  switch (m_model) {
    case FRICTIONLESS_POINT_CONTACT:
      S = math::MatrixX6(1, 6);
      S << 0, 0, 1, 0, 0, 0;
      break;

    case POINT_CONTACT_WITH_FRICTION:
      S              = math::MatrixX6(3, 6);
      S.leftCols(3)  = math::Matrix3d::Identity();
      S.rightCols(3) = math::Matrix3d::Zero();
      break;

    case SOFT_FINGER_CONTACT:
      S                   = math::MatrixX6::Zero(4, 6);
      S.block<3, 3>(0, 0) = math::Matrix3d::Identity();
      S(3, 5)             = 1.;
      break;

    default:
      CROBOT_LOG_ERROR("Unknown contact model.");
      break;
  }
  return S;
}

//==============================================================================
ContactModel Contact::get_contact_model() const {
  return m_model;
}
