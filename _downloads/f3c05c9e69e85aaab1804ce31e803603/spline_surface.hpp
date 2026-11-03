#pragma once

#include "crobot/math/surface.hpp"

namespace crobot::math {

class SplineSurface : public Surface {
public:
  /**
   * @brief Returns whether the surface is valid
   *
   * @return Whether valid
   */
  virtual bool is_valid() const = 0;

  /**
   * @brief Checks whether the surface is closed along u-direction
   *
   * @return  Whether closed along u-direction
   */
  virtual bool is_closed_along_u() const = 0;

  /**
   * @brief Checks whether the surface is closed along v-direction
   *
   * @return Whether closed along v-direction
   */
  virtual bool is_closed_along_v() const = 0;
};

} // namespace crobot::math

