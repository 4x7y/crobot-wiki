#pragma once

#include <memory>

#include "crobot/kdl/collision_contact.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::simulation {

class SimContact;
using ContactPtr      = std::shared_ptr<SimContact>;
using ConstContactPtr = std::shared_ptr<const SimContact>;

class SimContact : public kdl::CollisionContact {
public:
  static ContactPtr MakeShared(
      int geom1, int geom2, kdl::ContactModel contact_model, double mu);

  SimContact(int geom1, int geom2, kdl::ContactModel contact_model, double mu);

  bool is_active() const override;

  math::Vector3d get_xpos() const override;

  math::Matrix3d get_xmat() const override;

  math::Vector3d get_xfrc() const override;

  void reset();

  void update(
      const math::Vector3d& xpos,
      const math::Matrix3d& frame,
      const math::Vector3d& force,
      const math::Vector3d& torque);

private:
  int m_geom1_id;
  int m_geom2_id;
  bool m_is_active;

  math::Vector3d m_xpos;
  math::Matrix3d m_frame;

  math::Vector3d m_force;
  math::Vector3d m_torque;
};

} // namespace crobot::simulation

