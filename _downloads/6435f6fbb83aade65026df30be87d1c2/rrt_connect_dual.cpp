#include "crobot/planning/rrt_connect_dual.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
RRTConnectDual::RRTConnectDual(Planner::ConstStateSpacePtr state_space)
  : RRTDual(state_space) {
}

//==============================================================================
RRTConnectDual::~RRTConnectDual() {
}

//==============================================================================
std::string RRTConnectDual::get_algorithm_name() const {
  return "RRT Connect Connect";
}

//==============================================================================
bool RRTConnectDual::plan(
    const math::VectorXd& start, const math::VectorXd& goal) {
  m_time = ::std::chrono::steady_clock::now();

  m_begin[0] = add_vertex(m_trees[0], start);
  m_begin[1] = add_vertex(m_trees[1], goal);

  Tree* tree_a = &m_trees[0];
  Tree* tree_b = &m_trees[1];

  while ((std::chrono::steady_clock::now() - m_time) < get_duration()) {
    for (std::size_t j = 0; j < 2; ++j) {
      math::VectorXd chosen = m_state_space->sample();
      Neighbor aNearest     = nearest(*tree_a, chosen);
      Vertex a_connected    = connect(*tree_a, aNearest, chosen);

      if (nullptr != a_connected) {
        Neighbor bNearest
            = nearest(*tree_b, tree_a->vertex_properties(a_connected).qpos);
        Vertex b_connected = connect(
            *tree_b, bNearest, tree_a->vertex_properties(a_connected).qpos);

        if (nullptr != b_connected) {
          if (are_equal(
                  tree_a->vertex_properties(a_connected).qpos,
                  tree_b->vertex_properties(b_connected).qpos)) {
            m_end[0] = &m_trees[0] == tree_a ? a_connected : b_connected;
            m_end[1] = &m_trees[1] == tree_b ? b_connected : a_connected;
            return true;
          }
        }
      }

      std::swap(tree_a, tree_b);
    }
  }

  return false;
}
