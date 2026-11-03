#include "crobot/planning/rrt_dual.hpp"

using namespace crobot;
using namespace planning;

//==============================================================================
RRTDual::RRTDual(Planner::ConstStateSpacePtr state_space)
  : RRT(state_space, 2) {
}

//==============================================================================
RRTDual::~RRTDual() {
}

//==============================================================================
std::string RRTDual::get_algorithm_name() const {
  return "RRT Dual";
}

//==============================================================================
std::vector<math::VectorXd> RRTDual::get_path() {
  std::vector<math::VectorXd> path;
  Vertex vi = m_end[0];
  while (m_trees[0].vertex_indegree(vi) > 0) {
    path.push_back(m_trees[0].vertex_properties(vi).qpos);
    vi = m_trees[0].edge_source(*m_trees[0].get_vertex_inedges(vi).first);
  }
  path.push_back(m_trees[0].vertex_properties(vi).qpos);

  // reverese the path
  std::reverse(path.begin(), path.end());

  vi = m_trees[1].edge_source(*m_trees[1].get_vertex_inedges(m_end[1]).first);
  while (m_trees[1].vertex_indegree(vi) > 0) {
    path.push_back(m_trees[1].vertex_properties(vi).qpos);
    vi = m_trees[1].edge_source(*m_trees[1].get_vertex_inedges(vi).first);
  }
  path.push_back(m_trees[1].vertex_properties(vi).qpos);

  return path;
}

//==============================================================================
bool RRTDual::plan(const math::VectorXd& start, const math::VectorXd& goal) {
  m_time = ::std::chrono::steady_clock::now();

  m_begin[0] = add_vertex(m_trees[0], start);
  m_begin[1] = add_vertex(m_trees[1], goal);

  while ((std::chrono::steady_clock::now() - m_time) < get_duration()) {
    math::VectorXd chosen = m_state_space->sample();

    Neighbor nn      = nearest(m_trees[0], chosen);
    Vertex extended1 = extend(m_trees[0], nn, chosen);

    if (nullptr != extended1) {
      Neighbor nearest2 = nearest(m_trees[1], chosen);
      Vertex extended2  = extend(m_trees[1], nearest2, chosen);

      if (nullptr != extended2) {
        if (are_equal(
                m_trees[0].vertex_properties(extended1).qpos,
                m_trees[1].vertex_properties(extended2).qpos)) {
          m_end[0] = extended1;
          m_end[1] = extended2;
          return true;
        }
      }
    }
  }

  return false;
}
