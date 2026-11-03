#pragma once

#include <string>

#include "crobot/math/matrix.hpp"

namespace crobot::gui::internal {

class Shader {
public:
  // Load existing shader from id
  explicit Shader(math::Index id);
  // Load shader on construction from code
  Shader(
      const std::string& vertex_code,
      const std::string& fragment_code,
      const std::string& geometry_code = "");
  // Activate the shader
  void use();

  // Utility uniform functions
  void set_bool(const std::string& name, bool value) const;
  void set_int(const std::string& name, int value) const;
  void set_float(const std::string& name, float value) const;
  void set_vec2(const std::string& name, float x, float y) const;
  void set_vec3(const std::string& name, float x, float y, float z) const;
  void set_vec4(const std::string& name, float x, float y, float z, float w);

  // Eigen helpers
  void set_vec2(
      const std::string& name,
      const Eigen::Ref<const math::Vector2f>& value) const;
  void set_vec3(
      const std::string& name,
      const Eigen::Ref<const math::Vector3f>& value) const;
  void set_vec4(
      const std::string& name,
      const Eigen::Ref<const math::Vector4f>& value) const;
  void set_mat2(
      const std::string& name,
      const Eigen::Ref<const Eigen::Matrix2f>& mat) const;
  void set_mat3(
      const std::string& name,
      const Eigen::Ref<const Eigen::Matrix3f>& mat) const;
  void set_mat4(
      const std::string& name,
      const Eigen::Ref<const Eigen::Matrix4f>& mat) const;

  // GL shader id
  math::Index id;
};

} // namespace crobot::gui::internal

