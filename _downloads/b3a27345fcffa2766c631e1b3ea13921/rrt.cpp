#include "crobot/common/assert.hpp"
#include "crobot/math/linear_nearest_neighbors.hpp"
#include "crobot/math/random.hpp"
#include "crobot/math/state_space.hpp"
#include "crobot/planning/rrt.hpp"
#include "crobot/planning/viewer.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
RRT::RRT(Planner::ConstStateSpacePtr state_space) : RRT(state_space, 1) {
}

//==============================================================================
RRT::RRT(Planner::ConstStateSpacePtr state_space, std::size_t trees)
  : Planner(state_space),
    m_delta(1),
    m_epsilon(1.0e-3),
    m_begin(trees, nullptr),
    m_end(trees, nullptr),
    m_trees(trees),
    m_goal_bias(0.3) {
  set_duration(std::chrono::seconds(10));

  for (std::size_t i = 0; i < m_trees.size(); i++) {
    m_trees[i].graph_properties().nn
        = std::make_shared<math::LinearNearestNeighbors<NeighborNode>>(
            std::make_shared<RRTMetric>(state_space));
  }
}

//==============================================================================
RRT::~RRT() {
}

//==============================================================================
void RRT::set_delta(double delta) {
  m_delta = delta;
}

//==============================================================================
void RRT::set_epsilon(double epsilon) {
  m_epsilon = epsilon;
}

//==============================================================================
std::string RRT::get_algorithm_name() const {
  return "RRT";
}

//==============================================================================
std::size_t RRT::get_num_edges() const {
  std::size_t num_edges = 0;
  for (std::size_t i = 0; i < m_trees.size(); ++i) {
    num_edges += m_trees[i].num_edges();
  }
  return num_edges;
}

//==============================================================================
std::size_t RRT::get_num_vertices() const {
  std::size_t num_vertices = 0;
  for (std::size_t i = 0; i < m_trees.size(); ++i) {
    num_vertices += m_trees[i].num_vertices();
  }
  return num_vertices;
}

//==============================================================================
RRT::Edge RRT::add_edge(const Vertex& u, const Vertex& v, Tree& tree) {
  Edge e = tree.add_edge(u, v);

  if (m_viewer != nullptr) {
    m_viewer->draw_edge(
        tree.vertex_properties(u).qpos, tree.vertex_properties(v).qpos);
  }

  return e;
}

//==============================================================================
RRT::Vertex RRT::add_vertex(Tree& tree, const math::VectorXd& qpos) {
  VertexBundle bundle;
  bundle.index = tree.num_vertices() - 1;
  bundle.qpos  = qpos;

  Vertex v = tree.add_vertex(bundle);
  tree.graph_properties().nn->push(std::make_pair(qpos, v));

  if (m_viewer != nullptr) {
    m_viewer->draw_qpos(qpos);
  }

  return v;
}

//==============================================================================
bool RRT::are_equal(
    const math::VectorXd& lhs, const math::VectorXd& rhs) const {
  if (m_state_space->distance(lhs, rhs) > m_epsilon) {
    return false;
  }
  else {
    return true;
  }
}

//==============================================================================
RRT::Vertex RRT::connect(
    Tree& tree, const Neighbor& nearest, const math::VectorXd& chosen) {
  double distance = nearest.first;
  double step     = distance;
  bool reached    = false;

  if (step <= m_delta) {
    reached = true;
  }
  else {
    step = m_delta;
  }

  math::VectorXd last = m_state_space->interpolate(
      tree.vertex_properties(nearest.second).qpos, chosen, step / distance);

  if (m_state_space->is_state_valid(last)) {
    return nullptr;
  }

  while (!reached) {
    step += m_delta;

    if (step >= distance) {
      reached = true;
      step    = distance;
    }

    math::VectorXd next = m_state_space->interpolate(
        tree.vertex_properties(nearest.second).qpos, chosen, step / distance);

    if (m_state_space->is_state_valid(next)) {
      break;
    }

    last = next;
  }

  Vertex connected = add_vertex(tree, last);
  add_edge(nearest.second, connected, tree);
  return connected;
}

//==============================================================================
RRT::Vertex RRT::extend(
    Tree& tree, const Neighbor& nearest, const math::VectorXd& chosen) {
  double distance = nearest.first;
  double step     = std::min(distance, m_delta);

  math::VectorXd next = m_state_space->interpolate(
      tree.vertex_properties(nearest.second).qpos, chosen, step / distance);

  if (m_state_space->is_state_valid(next)) {
    Vertex extended = add_vertex(tree, next);
    add_edge(nearest.second, extended, tree);
    return extended;
  }

  return nullptr;
}

//==============================================================================
std::vector<math::VectorXd> RRT::get_path() {
  std::vector<Vertex> path = m_trees[0].backtrack_path(m_end[0]);
  std::vector<math::VectorXd> path_qpos;
  for (auto& v : path) {
    path_qpos.push_back(m_trees[0].vertex_properties(v).qpos);
  }
  return path_qpos;
}

//==============================================================================
RRT::Neighbor RRT::nearest(const Tree& tree, const math::VectorXd& chosen) {
  std::vector<NearestNeighbors::Neighbor> neighbors
      = tree.graph_properties().nn->nearest(
          std::make_pair<>(chosen, Vertex()), 1);
  return Neighbor(neighbors.front().first, neighbors.front().second.second);
}

//==============================================================================
void RRT::reset() {
  for (std::size_t i = 0; i < m_trees.size(); ++i) {
    m_trees[i].clear();
    m_trees[i].graph_properties().nn->clear();
    m_begin[i] = nullptr;
    m_end[i]   = nullptr;
  }
}

//==============================================================================
bool RRT::plan(const math::VectorXd& start, const math::VectorXd& goal) {
  m_time     = std::chrono::steady_clock::now();
  m_begin[0] = add_vertex(m_trees[0], start);
  while ((std::chrono::steady_clock::now() - m_time) < get_duration()) {

    math::VectorXd chosen;
    if (math::Random::uniform(0.0, 1.0) < m_goal_bias) {
      chosen = goal;
    }
    else {
      chosen = m_state_space->sample();
    }

    Neighbor nn     = nearest(m_trees[0], chosen);
    Vertex extended = extend(m_trees[0], nn, chosen);

    if (nullptr != extended) {
      if (are_equal(m_trees[0].vertex_properties(extended).qpos, goal)) {
        m_end[0] = extended;
        return true;
      }
    }
  }

  return false;
}
