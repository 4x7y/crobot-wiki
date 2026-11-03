#ifndef PATCH_NODE_HPP
#define PATCH_NODE_HPP

#include <memory>
#include <vector>

#include <Eigen/Core>

namespace crobot::patch_tree {

class PatchNode {
public:
  using Ptr      = std::shared_ptr<PatchNode>;
  using ConstPtr = std::shared_ptr<const PatchNode>;

  PatchNode(int id);

  void set_parent(PatchNode::ConstPtr parent);

  void set_vertices(const std::vector<Eigen::Vector3d>& vertices);

  void set_type(int type);

  int get_type() const;

  int get_id() const;

  PatchNode::ConstPtr get_parent() const;

  std::vector<PatchNode::Ptr> get_children() const;

  std::vector<Eigen::Vector3d> get_vertices() const;

  void add_child(PatchNode::Ptr child);

  bool is_leaf() const;

  bool is_root() const;

  std::vector<int> get_child_ids() const;

  void set_post_order(int post_order);

  int get_post_order() const;

private:
  int m_id;
  int m_post_order = -1;
  int m_type       = 1;
  // Children are owned downward. A strong parent pointer would retain the
  // entire hierarchy after PatchTree is destroyed.
  std::weak_ptr<const PatchNode> m_parent;
  std::vector<PatchNode::Ptr> m_children;
  std::vector<Eigen::Vector3d> m_vertices;
};
} // namespace crobot::patch_tree

#endif // PATCH_NODE_HPP
