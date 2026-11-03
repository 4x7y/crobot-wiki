#pragma once

#include <string>
#include <vector>

#include "crobot/math/matrix.hpp"

namespace crobot::common {

class VectorMapper {
public:
  VectorMapper(
      const std::vector<std::string> names_from,
      const std::vector<std::string> names_to);

  VectorMapper(const std::vector<VectorMapper>& mappers);

  math::VectorXd map(const math::VectorXd& v);

  math::VectorXd map(const std::vector<double>& v);

private:
  std::vector<std::string> m_names_from;
  std::vector<std::string> m_names_to;
  std::vector<std::size_t> m_indices;
};

} // namespace crobot::common

