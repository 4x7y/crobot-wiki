#pragma once

#include <memory>

#include "crobot/kdl/body_contact.hpp"
#include "crobot/kdl/collision_contact.hpp"
#include "crobot/kdl/frame.hpp"
#include "crobot/kdl/rigid_body.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::control {

class AdmittanceControlTarget : public kdl::WorldFrame {
public:
  using ConstPtr = std::shared_ptr<const AdmittanceControlTarget>;
  using Ptr      = std::shared_ptr<AdmittanceControlTarget>;

  AdmittanceControlTarget(
      kdl::ConstFramePtr desired_frame,
      kdl::BodyContact::ConstPtr contact,
      bool is_contact_active   = false,
      const math::Matrix3d& kp = math::Matrix3d::Identity() * 0.0003,
      const math::Matrix3d& ki = math::Matrix3d::Identity() * 0.0003);

  void set_desired_force(const math::Vector3d& sfrc);

  void set_kp(const math::Matrix3d& kp);

  void set_ki(const math::Matrix3d& ki);

  // Documentation inherited
  math::Matrix4d get_xform() const override;

  const std::string& get_name(size_t body_id) const;

private:
  kdl::ConstFramePtr m_desired_frame;
  kdl::BodyContact::ConstPtr m_contact;
  bool m_is_contact_active;
  math::Matrix3d m_kp;
  math::Matrix3d m_ki;
  math::Vector3d m_bfrc_desired;
};

} // namespace crobot::control

