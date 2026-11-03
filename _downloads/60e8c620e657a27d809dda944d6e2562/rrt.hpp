#pragma once

#include "crobot/math/graph.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/math/metric.hpp"
#include "crobot/math/nearest_neighbors.hpp"
#include "crobot/math/state_space.hpp"
#include "crobot/planning/path.hpp"
#include "crobot/planning/planner.hpp"
#include "crobot/planning/state_validator.hpp"

namespace crobot::planning {

/**
 * Rapidly-Exploring Random Trees.
 *
 * Steven M. LaValle. Rapidly-exploring random trees: A new tool for path
 * planning. Technical Report TR 98-11, Iowa State University, Ames, IA,
 * USA, October 1998.
 *
 * http://msl.cs.uiuc.edu/~lavalle/papers/Lav98c.pdf
 */
class RRT : public Planner {
public:
  RRT(ConstStateSpacePtr state_space);

  virtual ~RRT();

  double get_delta() const;

  void set_delta(double delta);

  double get_epsilon() const;

  void set_epsilon(double epsilon);

  std::string get_algorithm_name() const override;

  virtual std::size_t get_num_edges() const;

  virtual std::size_t get_num_vertices() const;

  virtual std::vector<math::VectorXd> get_path();

  virtual void reset();

  virtual bool plan(const math::VectorXd& start, const math::VectorXd& goal);

protected:
  RRT(ConstStateSpacePtr state_space, std::size_t num_trees);
  struct VertexBundle
  {
    std::size_t index;

    math::VectorXd qpos;
  };

  struct TreeBundle;

  using Tree   = math::Graph<VertexBundle, boost::no_property, TreeBundle>;
  using Vertex = typename Tree::Vertex;
  using Edge   = typename Tree::Edge;

  using NeighborNode = std::pair<math::VectorXd, Vertex>;

  using NearestNeighbors = math::NearestNeighbors<NeighborNode>;

  class RRTMetric : public math::Metric<NeighborNode> {
  public:
    RRTMetric(Planner::ConstStateSpacePtr state_space)
      : m_state_space(state_space) {
    }

    double distance(
        const NeighborNode& lhs, const NeighborNode& rhs) const override {
      return m_state_space->distance(lhs.first, rhs.first);
    }

  private:
    Planner::ConstStateSpacePtr m_state_space;
  };

  struct TreeBundle
  {
    std::shared_ptr<NearestNeighbors> nn;
  };

  using Neighbor = std::pair<double, Vertex>;

  virtual Edge add_edge(const Vertex& u, const Vertex& v, Tree& tree);

  virtual Vertex add_vertex(Tree& tree, const math::VectorXd& q);

  bool are_equal(const math::VectorXd& lhs, const math::VectorXd& rhs) const;

  virtual Vertex connect(
      Tree& tree, const Neighbor& nearest, const math::VectorXd& chosen);

  virtual Vertex extend(
      Tree& tree, const Neighbor& nearest, const math::VectorXd& chosen);

  virtual Neighbor nearest(const Tree& tree, const math::VectorXd& chosen);

  /// Configuration step size.
  double m_delta;

  /// Epsilon for configuration comparison.
  double m_epsilon;

  std::vector<Vertex> m_begin;

  std::vector<Vertex> m_end;

  std::vector<Tree> m_trees;

private:
  /// Probability that the goal is chosen as the next configuration.
  double m_goal_bias;
};

} // namespace crobot::planning

