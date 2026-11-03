#include "crobot/hardware/shadow_motor.hpp"

#include "crobot/common/assert.hpp"
#include "crobot/common/tabulate.hpp"

using namespace crobot;
using namespace hardware;

//==============================================================================
ShadowMotor::ShadowMotor() {
}

//==============================================================================
const std::string& ShadowMotor::get_name() const {
  return m_name;
}

//==============================================================================
int ShadowMotor::get_id() const {
  return m_id;
}

//==============================================================================
double ShadowMotor::get_qpos(const math::VectorXd& qpos_hand) const {
  double qpos = 0.0;
  for (const Target& target : m_targets) {
    qpos += qpos_hand[target.joint->get_qpos_addr()];
  }
  return qpos;
}

//==============================================================================
int ShadowMotor::get_active_joint_qpos_addr(
    const math::VectorXd& qpos_hand) const {
  double qpos = get_qpos(qpos_hand);
  for (const Target& target : m_targets) {
    if (target.range.first <= qpos && qpos <= target.range.second) {
      return target.joint->get_qpos_addr();
    }
  }
  CROBOT_ASSERT_ERROR(false, "No active joint found");
  return -1;
}

//==============================================================================
double ShadowMotor::get_torque_pwm2nm(
    double pwm, double qfrc_guess, const math::VectorXd& qpos_hand) const {
  math::VectorXd qpos_factor(m_strain_gauge.factors.size());
  for (std::size_t i = 0; i < m_strain_gauge.factors.size(); i++) {
    const Factor& factor = m_strain_gauge.factors[i];

    qpos_factor[i] = qpos_hand[factor.joint->get_qpos_addr()];
  }
  return m_strain_gauge.convert_pwm2nm(pwm, qfrc_guess, qpos_factor);
}

//==============================================================================
double ShadowMotor::StrainGauge::convert_pwm2nm(
    double pwm,
    double qfrc_guess,
    const math::VectorXd& qpos [[maybe_unused]]) const {
  // step 1:
  const double QFRC_THRESHOLD = 1e-4;
  if (std::abs(qfrc_guess) < QFRC_THRESHOLD) {
    return 0.0;
  }

  // step 2:
  pwm = std::max(pwm, 0.0);

  // step 3:
  double qfrc = 0.0;
  for (std::size_t i = 0; i < factors.size(); i++) {
    double k_s = qfrc_guess > 0.0 ? factors[i].k_s_pos : factors[i].k_s_neg;
    // double k_q = factors[i].k_q;
    // double b_q = factors[i].b_q;

    // qfrc = k_s * strain_gauge - k_s * (k_q * qpos + b_q)
    // qfrc += k_s * pwm - k_s * (k_q * qpos[i] + b_q);
    qfrc += k_s * pwm;
  }

  return qfrc;
}
