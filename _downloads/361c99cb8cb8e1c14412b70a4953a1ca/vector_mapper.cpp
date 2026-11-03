#include "crobot/common/vector_mapper.hpp"

#include <unordered_map>

#include "crobot/common/assert.hpp"

using namespace crobot;
using namespace common;

//==============================================================================
VectorMapper::VectorMapper(
    const std::vector<std::string> names_from,
    const std::vector<std::string> names_to)
  : m_names_from(names_from), m_names_to(names_to) {
  CROBOT_ASSERT_ERROR(
      m_names_from.size() == m_names_to.size(), "VectorMapper: size mismatch");

  m_indices.resize(m_names_from.size());
  std::unordered_map<std::string, std::size_t> name_to_index;
  for (std::size_t i = 0; i < m_names_to.size(); i++) {
    name_to_index[m_names_to[i]] = i;
  }
  for (std::size_t i = 0; i < m_names_from.size(); i++) {
    m_indices[i] = name_to_index[m_names_from[i]];
  }
}

//==============================================================================
VectorMapper::VectorMapper(const std::vector<VectorMapper>& mappers) {
  for (const auto& mapper : mappers) {
    m_names_from.insert(
        m_names_from.end(),
        mapper.m_names_from.begin(),
        mapper.m_names_from.end());
    m_names_to.insert(
        m_names_to.end(), mapper.m_names_to.begin(), mapper.m_names_to.end());
  }

  m_indices.resize(m_names_from.size());
  std::unordered_map<std::string, std::size_t> name_to_index;
  for (std::size_t i = 0; i < m_names_to.size(); i++) {
    name_to_index[m_names_to[i]] = i;
  }
  for (std::size_t i = 0; i < m_names_from.size(); i++) {
    m_indices[i] = name_to_index[m_names_from[i]];
  }
}

//==============================================================================
math::VectorXd VectorMapper::map(const math::VectorXd& v) {
  math::VectorXd v_out(m_names_to.size());
  for (std::size_t i = 0; i < m_names_to.size(); i++) {
    v_out[m_indices[i]] = v[i];
  }
  return v_out;
}

//==============================================================================
math::VectorXd VectorMapper::map(const std::vector<double>& v) {
  math::VectorXd v_out(m_names_to.size());
  for (std::size_t i = 0; i < m_names_to.size(); i++) {
    v_out[m_indices[i]] = v[i];
  }
  return v_out;
}
