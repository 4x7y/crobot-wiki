#pragma once

#include <memory>
#include <vector>

#include "crobot/math/matrix.hpp"

namespace crobot::simulation {

// forward declaration
class MJSim;

struct Marker
{
  // make Marker polymorphic
  virtual ~Marker() = default;
};

struct Box : public Marker
{
  Box(const math::Vector3d& xpos,
      const math::Matrix3d& xmat,
      const math::Vector3d& sz,
      const math::Vector4d& rgba)
    : xpos(xpos), xmat(xmat), sz(sz), rgba(rgba) {
  }
  math::Vector3d xpos;
  math::Matrix3d xmat;
  math::Vector3d sz;
  math::Vector4d rgba;
};

struct Line : public Marker
{
  Line(
      const math::Vector3d& from,
      const math::Vector3d& to,
      double radius,
      const math::Vector4d& rgba)
    : from(from), to(to), radius(radius), rgba(rgba) {
  }

  math::Vector3d from;
  math::Vector3d to;
  double radius;
  math::Vector4d rgba;
};

struct Arrow : public Marker
{
  Arrow(
      const math::Vector3d& from,
      const math::Vector3d& to,
      double sz,
      const math::Vector4d& rgba)
    : from(from), to(to), sz(sz), rgba(rgba) {
  }

  math::Vector3d from;
  math::Vector3d to;
  double sz;
  math::Vector4d rgba;
};

class MarkerManager {
public:
  void add_box_marker(
      const math::Vector3d& xpos,
      const math::Matrix3d& xmat,
      const math::Vector3d& size,
      const math::Vector4d& rgba) {
    m_markers.push_back(std::make_shared<Box>(xpos, xmat, size, rgba));
  }

  void add_cube_marker(
      const math::Vector3d& xpos, double size, const math::Vector4d& rgba) {
    m_markers.push_back(std::make_shared<Box>(
        xpos,
        math::Matrix3d::Identity(),
        math::Vector3d(size, size, size),
        rgba));
  }

  void add_line_marker(
      const math::Vector3d& from,
      const math::Vector3d& to,
      double radius,
      const math::Vector4d& rgba) {
    m_markers.push_back(std::make_shared<Line>(from, to, radius, rgba));
  }

  void add_arrow_marker(
      const math::Vector3d& from,
      const math::Vector3d& to,
      double radius,
      const math::Vector4d& rgba) {
    m_markers.push_back(std::make_shared<Arrow>(from, to, radius, rgba));
  }

  void add_frame_marker(const math::Matrix4d& xform, double size = 0.03) {
    add_frame_marker(xform.tform2trvec(), xform.tform2rotm(), size);
  }

  void add_frame_marker(
      const math::Vector3d& origin,
      const math::Matrix3d& xrot,
      double size = 0.03) {
    m_markers.push_back(std::make_shared<Arrow>(
        origin,
        origin + xrot.col(0) * size,
        0.001,
        math::Vector4d(1, 0, 0, 1))); // Red - X
    m_markers.push_back(std::make_shared<Arrow>(
        origin,
        origin + xrot.col(1) * size,
        0.001,
        math::Vector4d(0, 1, 0, 1))); // Green - Y
    m_markers.push_back(std::make_shared<Arrow>(
        origin,
        origin + xrot.col(2) * size,
        0.001,
        math::Vector4d(0, 0, 1, 1))); // Blue - Z
  }

  void clear() {
    m_markers.clear();
  }

  const std::vector<std::shared_ptr<Marker>>& get_markers() {
    return m_markers;
  }

private:
  std::vector<std::shared_ptr<Marker>> m_markers;

  friend class MJSim;
};

} // namespace crobot::simulation

