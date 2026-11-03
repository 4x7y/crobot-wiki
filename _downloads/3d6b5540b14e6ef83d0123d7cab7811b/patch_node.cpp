
#include "crobot/patch_tree/patch_node.hpp"

using namespace crobot::patch_tree;

//==============================================================================
PatchNode::PatchNode(int id) : m_id(id) {
}

//==============================================================================
void PatchNode::set_parent(PatchNode::ConstPtr parent) {
  m_parent = parent;
}

//==============================================================================
void PatchNode::set_vertices(const std::vector<Eigen::Vector3d>& vertices) {
  m_vertices = vertices;
}

//==============================================================================
int PatchNode::get_id() const {
  return m_id;
}

//==============================================================================

PatchNode::ConstPtr PatchNode::get_parent() const {
  return m_parent.lock();
}

//==============================================================================
std::vector<PatchNode::Ptr> PatchNode::get_children() const {
  return m_children;
}

//==============================================================================

std::vector<Eigen::Vector3d> PatchNode::get_vertices() const {
  return m_vertices;
}

//==============================================================================
void PatchNode::add_child(PatchNode::Ptr child) {
  m_children.push_back(child);
}

//==============================================================================
bool PatchNode::is_leaf() const {
  return m_children.empty();
}

//==============================================================================
std::vector<int> PatchNode::get_child_ids() const {
  std::vector<int> child_ids;
  for (size_t i = 0; i < m_children.size(); i++) {
    child_ids.push_back(m_children[i]->get_id());
  }
  return child_ids;
}

//==============================================================================
bool PatchNode::is_root() const {
  return m_parent.expired();
}

//==============================================================================
void PatchNode::set_type(int type) {
  m_type = type;
}

//==============================================================================
int PatchNode::get_type() const {
  return m_type;
}

//==============================================================================
void PatchNode::set_post_order(int post_order) {
  m_post_order = post_order;
}

//==============================================================================
int PatchNode::get_post_order() const {
  return m_post_order;
}
