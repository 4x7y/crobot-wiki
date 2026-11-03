#pragma once

#include <memory>
#include <vector>

namespace crobot::kdl {

class Transmission {
public:
  Transmission()                    = default;
  virtual ~Transmission()           = default;
  Transmission(const Transmission&) = default;
  Transmission& operator=(const Transmission&) = default;

  /**
   * @brief Get the joint configuration indices in the RigidBodyTree
   * configuration vector @c qpos .
   */
  virtual std::vector<std::pair<int, int>> get_qpos_addr() const = 0;

  /**
   * @brief Get the joint velocity (acceleration) indices in the RigidBodyTree
   * joint velocity (acceleration) vector @c qvel ( @c qacc ).
   */
  virtual std::vector<std::pair<int, int>> get_qvel_addr() const = 0;

  /**
   * @brief Get the transmission coefficients.
   */
  virtual std::vector<double> get_transmission_coeffs() const = 0;
};

using TransmissionPtr      = std::shared_ptr<Transmission>;
using ConstTransmissionPtr = std::shared_ptr<const Transmission>;

} // namespace crobot::kdl

