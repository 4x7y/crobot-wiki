#pragma once

#include <vector>

#include "crobot/math/array.hpp"
#include "crobot/math/matrix.hpp"

namespace crobot::math {

class Geometry {
public:
  using Ptr      = std::shared_ptr<Geometry>;
  using ConstPtr = std::shared_ptr<const Geometry>;

  virtual ~Geometry() = default;
};

} // namespace crobot::math

