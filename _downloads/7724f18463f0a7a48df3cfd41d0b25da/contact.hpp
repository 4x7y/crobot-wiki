#pragma once

#include <memory>

#include "crobot/math/matrix.hpp"

namespace crobot::kdl {

enum ContactModel
{
  FRICTIONLESS_POINT_CONTACT,
  POINT_CONTACT_WITH_FRICTION,
  SOFT_FINGER_CONTACT,
};

class Contact {
public:
  /**
   * @brief Construct a new Contact object
   *
   * @param contact_model The ContactModel of this contact.
   * @param mu The Columb's friction coefficient at the contact.
   *
   * Static friction coefficients for some common materials.
   * (Source: CRC Handbook of Chemistry and Physics)
   *
   *   Steel on steel        0.58          Wood on wood     0.25-0.5
   *   Polyethylene on steel 0.3–0.35      Wood on metals   0.2-0.6
   *   Polyethylene on self  0.5           Wood on leather  0.3–0.4
   *   Rubber on solids      1–4           Leather on metal 0.6
   */
  Contact(ContactModel contact_model, double mu = 1.0);

  /**
   * @brief Get the Columb's friction coefficient.
   */
  double get_mu() const;

  /**
   * @brief Get the number of wrench basis.
   */
  int get_num_wrench_basis() const;

  /**
   * @brief Get the all wrench basis.
   *
   * This equivalents to the traspose of the selection matrix:
   *
   *                            W = S.T
   */
  math::MatrixX6 get_wrench_basis() const;

  /**
   * @brief Get the selection matrix according to the given conact model.
   *
   * For firctionless point contact, the selection matrix is defined as
   *
   *                       S = [0 0 1 0 0 0]
   *
   * which only takes the contact force along the contact frame normal axis.
   *
   * For point conact with friction (PCWF) model, it is defined as
   *
   *                           [1 0 0 0 0 0]
   *                       S = [0 1 0 0 0 0]
   *                           [0 0 1 0 0 0]
   *
   * For softfinger model, it is defined as
   *
   *                           [1 0 0 0 0 0]
   *                       S = [0 1 0 0 0 0]
   *                           [0 0 1 0 0 0]
   *                           [0 0 0 0 0 1]
   */
  math::MatrixX6 get_selection_matrix() const;

  ContactModel get_contact_model() const;

private:
  ContactModel m_model;
  double m_mu;
};

using ContactPtr      = std::shared_ptr<Contact>;
using ConstContactPtr = std::shared_ptr<const Contact>;

} // namespace crobot::kdl

