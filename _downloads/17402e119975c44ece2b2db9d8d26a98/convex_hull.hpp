#ifndef CONVEX_HULL_HPP
#define CONVEX_HULL_HPP

#include "crobot/common/common.hpp"
#include "crobot/grasping/disk.hpp"
#include "crobot/grasping/vector_set.hpp"
#include "crobot/math/math.hpp"

namespace crobot {

class ConvexHull {
public:
  using Ptr      = std::shared_ptr<ConvexHull>;
  using ConstPtr = std::shared_ptr<const ConvexHull>;

  // Callback function type for visualization
  // Parameters: lines (start, end pairs) and points to visualize
  using VisualizationCallback = std::function<void(
      const std::vector<std::pair<math::Vector3d, math::Vector3d>>& lines,
      const std::vector<math::Vector3d>& points)>;

  ConvexHull(
      VectorSet::Ptr vertex_set, std::vector<int> disk_index, int num_disks);

  ConvexHull(
      VectorSet::Ptr vertex_set,
      std::vector<int> disk_index,
      int num_disks,
      VisualizationCallback visualization_callback);

  int get_min_index(double lb) const;
  double get_dist2origin(int index) const;
  math::VectorXd get_normal(int index) const;

  void add_vertex(
      const math::VectorXd& vertex, int disk_index, int facet_index);

  double get_max_index() const;

  bool is_empty() const;

  bool exceeds(int num_facets) const;

private:
  // bool is_shared_edge(
  //     const VectorSet::Ptr edge,
  //     const VectorSet::Ptr f1,
  //     const VectorSet::Ptr f2) const;

  bool contains_facet(
      const std::vector<VectorSet::Ptr>& facets, const VectorSet::Ptr f) const;

  bool contains_vertex(const math::VectorXd& vertex) const;

  VectorSet::Ptr get_plane_facet(
      const VectorSet::Ptr set, const math::VectorXd& ref_vec) const;

  void remove_facet(int index);

  void add_facet(VectorSet::Ptr facet, const std::string& add_msg = "");

  std::pair<math::VectorXd, double> get_facet_normal_dist(
      VectorSet::Ptr facet) const;

  void print_facet(const VectorSet::Ptr facet, const std::string& name) const;

  bool is_facet_on_CH(const VectorSet::Ptr edge) const;

  void plot_facets(const math::VectorXd& point) const;

  std::vector<VectorSet::Ptr> m_facets;  // 5D structure with 6 vectors
  std::vector<math::VectorXd> m_normals; // 6D vectors
  std::vector<double> m_dist2origin;     // distance to origin

  std::vector<Disk::Ptr> m_disks;

  math::VectorXd m_special_inner;
  VectorSet::Ptr m_vertex_set;

  VisualizationCallback m_visualization_callback;
};
} // namespace crobot

#endif // CONVEX_HULL_HPP
