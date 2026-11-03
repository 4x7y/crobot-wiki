#ifndef DISK_HPP
#define DISK_HPP

#include "crobot/common/common.hpp"
#include "crobot/math/math.hpp"

#include "crobot/grasping/vector_set.hpp"

namespace crobot {

class Disk {
public:
  using Ptr      = std::shared_ptr<Disk>;
  using ConstPtr = std::shared_ptr<const Disk>;

  Disk();

  bool is_plane_construted() const;

  void consttruct_plane();

  math::Vector2d get_plane_coord(const math::VectorXd& vertex) const;

  // disappearing vertices in CW order

  std::vector<math::VectorXd> add_vertex(const math::VectorXd& vertex);

  void print_vertices() const;

  void print_plane_coords() const;

  int get_num_vertices() const;

  bool contains(const math::VectorXd& vertex) const;

  bool is_on(const math::VectorXd& vertex) const;

  std::vector<math::VectorXd> get_vertices() const;

  math::VectorXd generate_random_point() const;

private:
  bool is_between(const math::Vector2d& test, int i, int j) const;

  std::vector<math::VectorXd> m_vertices;
  std::vector<math::Vector2d> m_plane_coords;

  // local coordinate system
  math::VectorXd m_origin;
  math::VectorXd m_t1;
  math::VectorXd m_t2;
};
} // namespace crobot

#endif // DISK_HPP
