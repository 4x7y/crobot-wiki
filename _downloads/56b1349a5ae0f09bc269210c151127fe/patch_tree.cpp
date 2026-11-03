#include "crobot/patch_tree/patch_tree.hpp"

#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <limits>
#include <set>
#include <stdexcept>

#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

#include "crobot/geometry/patch_mesh_data.hpp"

using namespace crobot::patch_tree;
using crobot::geometry::PatchMeshData;

//==============================================================================
PatchTree::PatchTree(
    std::vector<NodeData> nodes, const std::vector<PatchEdge>& leaf_edges) {
  if (nodes.empty()
      || nodes.size() > static_cast<size_t>(std::numeric_limits<int>::max())) {
    throw std::invalid_argument("A generated patch tree needs a root node");
  }
  const int count = static_cast<int>(nodes.size());
  if (nodes[0].parent_id != -1 || nodes[0].type != 1) {
    throw std::invalid_argument(
        "Generated root must have parent -1 and type 1");
  }

  std::vector<std::vector<int>> children(count);
  auto valid_status = [](BoundStatus status) {
    return status == BoundStatus::NotEvaluated
           || status == BoundStatus::Satisfied
           || status == BoundStatus::Indivisible
           || status == BoundStatus::RefinementLimit;
  };
  for (int id = 0; id < count; ++id) {
    auto& node = nodes[id];
    if (id != 0) {
      if (node.parent_id < 0 || node.parent_id >= count
          || node.parent_id == id) {
        throw std::invalid_argument("Invalid generated patch parent ID");
      }
      children[node.parent_id].push_back(id);
    }
    if (node.type < 1 || node.type > 3
        || (node.type == 2 && node.vertices.size() < 3)
        || (node.type == 3 && node.vertices.size() < 4)) {
      throw std::invalid_argument("Invalid generated patch type or descriptor");
    }
    if (id != 0) {
      const int parent_type = nodes[node.parent_id].type;
      if (node.type < parent_type || (node.type == 3 && parent_type == 1)) {
        throw std::invalid_argument(
            "Patch types must follow component, normal, torque order");
      }
    }
    for (const auto& vertex : node.vertices) {
      if (!vertex.allFinite()) {
        throw std::invalid_argument("Generated patch descriptor is not finite");
      }
    }
    if (!valid_status(node.curvature_status)
        || !valid_status(node.normal_status)
        || !valid_status(node.torque_status)
        || !std::isfinite(node.total_curvature) || node.total_curvature < 0.0
        || !std::isfinite(node.max_normal_angle) || node.max_normal_angle < 0.0
        || node.max_normal_angle > std::acos(-1.0)
        || !std::isfinite(node.torque_span) || node.torque_span < 0.0
        || node.curvature_class < -1 || node.curvature_class > 4
        || node.monotone_region_id < -1
        || !std::isfinite(node.uncertain_area_fraction)
        || node.uncertain_area_fraction < 0 || node.uncertain_area_fraction > 1
        || static_cast<int>(node.stage) < 0
        || static_cast<int>(node.stage) > static_cast<int>(Stage::Torque)) {
      throw std::invalid_argument("Invalid generated patch bound diagnostics");
    }

    std::sort(node.face_ids.begin(), node.face_ids.end());
    if ((!node.face_ids.empty() && node.face_ids.front() < 0)
        || std::adjacent_find(node.face_ids.begin(), node.face_ids.end())
               != node.face_ids.end()
        || (node.face_ids.empty() && (id != 0 || count != 1))) {
      throw std::invalid_argument(
          "Generated patch faces must be nonnegative, unique and nonempty");
    }
    std::sort(node.source_face_ids.begin(), node.source_face_ids.end());
    if (!node.source_face_ids.empty() && node.source_face_ids.front() < 0) {
      throw std::invalid_argument("Invalid original surface face ID");
    }
    node.source_face_ids.erase(
        std::unique(node.source_face_ids.begin(), node.source_face_ids.end()),
        node.source_face_ids.end());
    if (node.face_ids.empty() && !node.source_face_ids.empty()) {
      throw std::invalid_argument(
          "An empty surface cannot have source face membership");
    }
  }

  // Iterative traversal supports arbitrary parent/child index ordering and
  // rejects disconnected parent cycles without following their parent links.
  std::vector<int> post_order;
  std::vector<std::pair<int, bool>> pending{{0, false}};
  while (!pending.empty()) {
    const auto current = pending.back();
    pending.pop_back();
    if (current.second) {
      post_order.push_back(current.first);
      continue;
    }
    pending.emplace_back(current.first, true);
    const auto& child_ids = children[current.first];
    for (auto it = child_ids.rbegin(); it != child_ids.rend(); ++it) {
      pending.emplace_back(*it, false);
    }
  }
  if (post_order.size() != nodes.size()) {
    throw std::invalid_argument(
        "Generated patch records contain a disconnected cycle");
  }

  for (int id : post_order) {
    if (children[id].empty()) {
      continue;
    }
    std::vector<int> face_union;
    std::vector<int> source_union;
    for (int child : children[id]) {
      face_union.insert(
          face_union.end(),
          nodes[child].face_ids.begin(),
          nodes[child].face_ids.end());
      source_union.insert(
          source_union.end(),
          nodes[child].source_face_ids.begin(),
          nodes[child].source_face_ids.end());
    }
    std::sort(face_union.begin(), face_union.end());
    if (face_union != nodes[id].face_ids) {
      throw std::invalid_argument(
          "Children must exactly partition their parent's prepared faces");
    }
    std::sort(source_union.begin(), source_union.end());
    source_union.erase(
        std::unique(source_union.begin(), source_union.end()),
        source_union.end());
    if (source_union != nodes[id].source_face_ids) {
      throw std::invalid_argument(
          "Child source-face union must match its parent");
    }
  }

  std::vector<std::vector<int>> adjacency(count);
  for (const auto& edge : leaf_edges) {
    for (int endpoint : {edge.first, edge.second}) {
      if (endpoint < 0 || endpoint >= count || !children[endpoint].empty()
          || nodes[endpoint].face_ids.empty()) {
        throw std::invalid_argument(
            "Generated adjacency endpoint must be a nonempty leaf");
      }
    }
    if (edge.first != edge.second) {
      adjacency[edge.first].push_back(edge.second);
      adjacency[edge.second].push_back(edge.first);
    }
  }

  // Connected leaves supplied by the generator imply a connected internal
  // patch iff its descendant leaves form a connected induced subgraph.
  // Root is allowed to represent multiple disconnected object components.
  std::vector<std::vector<int>> descendant_leaves(count);
  std::vector<int> member(count, -1), visited(count, -1), leaf_ids;
  for (int id : post_order) {
    if (children[id].empty()) {
      if (!nodes[id].face_ids.empty()) {
        descendant_leaves[id].push_back(id);
        leaf_ids.push_back(id);
      }
      continue;
    }
    auto& leaves            = descendant_leaves[id];
    size_t descendant_count = 0;
    for (int child : children[id]) {
      descendant_count += descendant_leaves[child].size();
    }
    leaves.reserve(descendant_count);
    for (int child : children[id]) {
      leaves.insert(
          leaves.end(),
          descendant_leaves[child].begin(),
          descendant_leaves[child].end());
      // Once a parent has consumed this list no other node needs it. Retain
      // only the traversal frontier instead of every ancestor's leaf list.
      std::vector<int>().swap(descendant_leaves[child]);
    }
    if (id == 0 || leaves.size() < 2) {
      continue;
    }
    for (int leaf : leaves) {
      member[leaf] = id;
    }
    std::vector<int> reached{leaves.front()};
    visited[leaves.front()] = id;
    for (size_t cursor = 0; cursor < reached.size(); ++cursor) {
      for (int neighbor : adjacency[reached[cursor]]) {
        if (member[neighbor] == id && visited[neighbor] != id) {
          visited[neighbor] = id;
          reached.push_back(neighbor);
        }
      }
    }
    if (reached.size() != leaves.size()) {
      throw std::invalid_argument(
          "Generated internal patch is disconnected in leaf adjacency");
    }
  }

  m_nodes.reserve(nodes.size());
  for (int id = 0; id < count; ++id) {
    m_nodes.push_back(std::make_shared<PatchNode>(id));
    m_nodes.back()->set_type(nodes[id].type);
    m_nodes.back()->set_vertices(nodes[id].vertices);
  }
  m_root = m_nodes[0];
  for (int id = 1; id < count; ++id) {
    m_nodes[id]->set_parent(m_nodes[nodes[id].parent_id]);
    m_nodes[nodes[id].parent_id]->add_child(m_nodes[id]);
  }
  for (size_t position = 0; position < post_order.size(); ++position) {
    m_nodes[post_order[position]]->set_post_order(static_cast<int>(position));
  }
  m_node_data = std::move(nodes);
  build_level_graphs(leaf_ids, leaf_edges);
}

//==============================================================================
PatchTree::PatchTree(const std::string& folder, int num, double scale)
  : m_folder(folder) {

  m_nodes.push_back(std::make_shared<PatchNode>(0));
  m_root = m_nodes[0];
  m_root->set_type(1);

  for (int i = 0; i < num; i++) {
    m_nodes.push_back(std::make_shared<PatchNode>(i + 1));
    m_nodes[i + 1]->set_parent(m_nodes[0]);
    m_nodes[i + 1]->set_type(1);
    m_nodes[0]->add_child(m_nodes[i + 1]);
  }

  for (int i = 0; i < num; i++) {
    std::string filename
        = folder + "/" + std::to_string(i) + "/patch_tree.yaml";
    YAML::Node data = YAML::LoadFile(filename);

    int current_size = m_nodes.size();
    m_partitions.emplace_back(current_size, data.size());

    for (size_t i = 0; i < data.size(); i++) {
      m_nodes.push_back(std::make_shared<PatchNode>(current_size + i));
    }

    for (const auto& node : data) {
      int id        = node["id"].as<int>() + current_size - 1;
      int parent_id = node["parent"].as<int>();
      if (parent_id != 0)
        parent_id += current_size - 1;
      else
        parent_id = i + 1;
      int type = node["type"].as<int>();

      std::vector<Eigen::Vector3d> vertices;
      for (const auto& vertex : node["vertices"]) {
        vertices.push_back(Eigen::Vector3d(
            vertex[0].as<double>() * scale,
            vertex[1].as<double>() * scale,
            vertex[2].as<double>() * scale));
      }

      m_nodes[id]->set_parent(m_nodes[parent_id]);
      m_nodes[id]->set_type(type);
      m_nodes[id]->set_vertices(vertices);
      m_nodes[parent_id]->add_child(m_nodes[id]);
    }
  }

  // set post order
  int post_order = 0;
  std::function<void(PatchNode::Ptr)> set_post_order
      = [&](PatchNode::Ptr node) {
          for (auto child : node->get_children()) {
            set_post_order(child);
          }
          node->set_post_order(post_order++);
        };
  set_post_order(m_root);

  std::cout << "PatchTree is created with " << m_nodes.size() << " nodes"
            << std::endl;
}

//==============================================================================
PatchNode::ConstPtr PatchTree::get_root() const {
  return m_root;
}

//==============================================================================
PatchNode::ConstPtr PatchTree::get_node(int node_id) const {
  return m_nodes.at(node_id);
}

//==============================================================================
int PatchTree::get_num_nodes() const {
  return static_cast<int>(m_nodes.size());
}

//==============================================================================
bool PatchTree::has_face_membership() const {
  return !m_node_data.empty();
}

//==============================================================================
const PatchTree::NodeData& PatchTree::get_node_data(int node_id) const {
  if (!has_face_membership()) {
    throw std::logic_error(
        "Saved patch trees have no in-memory face membership");
  }
  return m_node_data.at(node_id);
}

//==============================================================================
const std::vector<int>& PatchTree::get_face_ids(int node_id) const {
  return get_node_data(node_id).face_ids;
}

//==============================================================================
int PatchTree::get_num_levels() const {
  return static_cast<int>(m_level_graphs.size());
}

int PatchTree::get_level_depth(int level) const {
  return m_level_depths.at(level);
}

//==============================================================================
const PatchTree::LevelGraph& PatchTree::get_level_graph(int level) const {
  return m_level_graphs.at(level);
}

//==============================================================================
void PatchTree::build_level_graphs(
    const std::vector<int>& leaf_ids,
    const std::vector<PatchEdge>& leaf_edges) {
  // Paths are root-to-leaf; repeating the last entry carries a shallow leaf
  // through the remaining levels without changing the underlying tree.
  std::map<int, std::vector<int>> paths;
  size_t num_levels = 1;
  for (int id : leaf_ids) {
    auto node = get_node(id);
    if (!node->is_leaf()) {
      throw std::invalid_argument("Level graph input must contain only leaves");
    }
    if (paths.count(id)) {
      continue;
    }
    auto& path = paths[id];
    while (node) {
      path.push_back(node->get_id());
      if (path.size() > m_nodes.size()) {
        throw std::invalid_argument("Patch parent links contain a cycle");
      }
      node = node->get_parent();
    }
    std::reverse(path.begin(), path.end());
    if (path.front() != m_root->get_id()) {
      throw std::invalid_argument(
          "Active leaf is not attached to the tree root");
    }
    num_levels = std::max(num_levels, path.size());
  }

  for (const auto& edge : leaf_edges) {
    if (!paths.count(edge.first) || !paths.count(edge.second)) {
      throw std::invalid_argument("Edge endpoint is not an active leaf");
    }
  }

  // A nested cut can only split patches, never merge them: equal counts mean
  // exactly the same partition, even when the semantic node IDs have changed.
  std::vector<std::set<int>> active(num_levels);
  for (auto& [id, path] : paths) {
    (void)id;
    path.resize(num_levels, path.back());
    for (size_t depth = 0; depth < num_levels; ++depth)
      active[depth].insert(path[depth]);
  }
  std::vector<int> depths{0};
  for (size_t depth = 1; depth < num_levels; ++depth)
    if (active[depth].size() > active[depths.back()].size())
      depths.push_back(static_cast<int>(depth));
  const int last = static_cast<int>(num_levels - 1);
  if (depths.back() != last) {
    // The final cut must name actual terminal planning nodes, retaining their
    // bound descriptors/statuses. Replace the start of its unchanged plateau.
    if (depths.size() == 1)
      depths.push_back(last);
    else
      depths.back() = last;
  }
  std::vector<LevelGraph> levels(depths.size());
  for (auto& entry : paths) {
    auto& path = entry.second;
    for (size_t level = 0; level < depths.size(); ++level) {
      auto& patch = levels[level][path[depths[level]]];
      if (level > 0) {
        patch.parent_id = path[depths[level - 1]];
      }
    }
  }

  // Every next-level node has exactly one previous-level parent. Add these
  // links once, rather than once per descendant leaf (which can be large).
  for (size_t level = 1; level < depths.size(); ++level) {
    for (const auto& child : levels[level]) {
      levels[level - 1]
          .at(child.second.parent_id)
          .child_ids.push_back(child.first);
    }
  }

  for (const auto& edge : leaf_edges) {
    const auto& first  = paths.at(edge.first);
    const auto& second = paths.at(edge.second);
    for (size_t level = 0; level < depths.size(); ++level) {
      const int depth = depths[level];
      if (first[depth] != second[depth]) {
        levels[level].at(first[depth]).neighbor_ids.push_back(second[depth]);
        levels[level].at(second[depth]).neighbor_ids.push_back(first[depth]);
      }
    }
  }

  auto deduplicate = [](std::vector<int>& ids) {
    std::sort(ids.begin(), ids.end());
    ids.erase(std::unique(ids.begin(), ids.end()), ids.end());
  };
  for (auto& level : levels) {
    for (auto& entry : level) {
      deduplicate(entry.second.neighbor_ids);
    }
  }
  m_level_graphs.swap(levels);
  m_level_depths.swap(depths);
}

//==============================================================================
void PatchTree::build_level_graphs() {
  if (has_face_membership()) {
    throw std::logic_error(
        "Generated trees require explicit leaf adjacency when rebuilding "
        "graphs");
  }
  std::vector<int> sizes;
  for (const auto& partition : m_partitions) {
    sizes.push_back(partition.second);
  }
  build_level_graphs(*PatchMeshData::load(m_folder, sizes));
}

//==============================================================================
void PatchTree::build_level_graphs(const PatchMeshData& mesh) {
  const auto& partitions = mesh.get_partitions();
  if (partitions.size() != m_partitions.size()) {
    throw std::invalid_argument("Mesh partitions do not match the patch tree");
  }
  for (size_t i = 0; i < partitions.size(); ++i) {
    if (partitions[i].first_patch_id != m_partitions[i].first
        || partitions[i].patch_count != m_partitions[i].second) {
      throw std::invalid_argument("Mesh patch IDs do not match the patch tree");
    }
  }
  const auto adjacency = mesh.get_patch_adjacency();
  for (int id : adjacency.patch_ids) {
    if (!get_node(id)->is_leaf()) {
      throw std::runtime_error("Face label references an internal patch");
    }
  }
  build_level_graphs(adjacency.patch_ids, adjacency.edges);
}

//==============================================================================
std::vector<std::vector<int>> PatchTree::get_next_level_combination(
    const std::vector<int>& comb) const {
  std::vector<std::vector<int>> ret;

  // find first index that has children
  size_t i = 0;
  for (; i < comb.size(); i++) {
    if (!get_node(comb[i])->is_leaf()) {
      break;
    }
  }

  if (i == comb.size()) {
    return ret; // no next level combination
  }

  auto children = get_node(comb[i])->get_children();
  for (auto child : children) {
    std::vector<int> new_comb = comb;
    new_comb[i]               = child->get_id();
    ret.push_back(new_comb);
  }

  return ret;
}

//==============================================================================
std::vector<std::vector<int>> PatchTree::get_next_expand_combination(
    const std::vector<int>& comb, int expand_id) const {
  std::vector<std::vector<int>> ret_comb;
  if (comb.empty()) {
    return ret_comb;
  }
  if (expand_id < 0 || expand_id >= static_cast<int>(comb.size())) {
    throw std::out_of_range("Combination expansion index is out of range");
  }
  for (size_t i = 0; i < comb.size(); i++) {
    int checking_id = (expand_id + i) % comb.size();
    if (!get_node(comb[checking_id])->is_leaf()) {
      // not a leaf, replace it with its children
      auto children = get_node(comb[checking_id])->get_children();
      for (auto child : children) {
        std::vector<int> new_comb = comb;
        new_comb[checking_id]     = child->get_id();
        if (is_valid_combination(new_comb))
          ret_comb.push_back(new_comb);
      }
      break;
    }
  }

  return ret_comb;
}

//==============================================================================
bool PatchTree::is_valid_combination(const std::vector<int>& comb) const {

  for (int id : comb) {
    get_node(id); // Check singleton combinations as well.
  }
  for (size_t i = 0; i + 1 < comb.size(); i++) {
    const auto node1 = get_node(comb[i]);
    const auto node2 = get_node(comb[i + 1]);
    if (node1->get_post_order() > node2->get_post_order()) {
      return false;
    }
  }

  return true;
}

//==============================================================================
std::vector<Eigen::Vector3d> PatchTree::get_vertices_in_combination(
    const std::vector<int>& comb) const {
  std::vector<Eigen::Vector3d> ret;

  int desired_type = 3;
  for (size_t i = 0; i < comb.size(); i++) {

    if (get_node(comb[i])->get_type() <= 1)
      throw std::invalid_argument(
          "Coarse nodes have no normal/torque descriptor");
    if (get_node(comb[i])->get_type() == 2) {
      desired_type = 2; // if any of the nodes is of type 2, then the desired
                        // type is 2
      break;
    }
  }

  for (size_t i = 0; i < comb.size(); i++) {
    PatchNode::ConstPtr node = get_node(comb[i]);
    while (node->get_type() != desired_type) {
      node = node->get_parent(); // go up to the desired type
    }

    auto vertices = node->get_vertices();
    for (size_t j = 0; j < vertices.size(); j++) {
      ret.push_back(vertices[j]);
    }
  }

  return ret;
}

//==============================================================================
bool PatchTree::is_valid(const std::vector<int>& comb) const {

  std::vector<Eigen::Vector3d> vertices = get_vertices_in_combination(comb);

  for (size_t i = 0; i < vertices.size(); i++) {
    for (size_t j = i + 1; j < vertices.size(); j++) {
      Eigen::Vector3d v1 = vertices[i];
      Eigen::Vector3d v2 = vertices[j];
      Eigen::Vector3d n  = v1.cross(v2);

      if (n.norm() < 1e-6) {
        continue;
      }

      n.normalize();
      bool valid = false;
      for (size_t k = 0; k < vertices.size(); k++) {
        if (k == i || k == j) {
          continue;
        }
        Eigen::Vector3d v3 = vertices[k];
        if (v3.dot(n) < 0.0) {
          valid = true;
          break;
        }
      }

      if (!valid) {
        return false;
      }
    }
  }

  return true;
}

//==============================================================================
std::vector<int> PatchTree::get_child_ids(int node_id) const {
  return get_node(node_id)->get_child_ids();
}

//==============================================================================
bool PatchTree::is_checkable(const std::vector<int>& comb) const {
  for (size_t i = 0; i < comb.size(); i++) {
    if (get_node(comb[i])->get_type() <= 1) {
      return false;
    }
  }
  return true;
}

//==============================================================================
int PatchTree::get_height() const {
  // int height = 0;
  // for (size_t i = 0; i < m_nodes.size(); i++) {
  //   int h                    = 0;
  //   PatchNode::ConstPtr node = m_nodes[i];
  //   while (!node->is_root()) {
  //     node = node->get_parent();
  //     h++;
  //   }
  //   if (h > height) {
  //     height = h;
  //   }
  // }
  return 2 + get_l1() + get_l2();
}

//==============================================================================
int PatchTree::get_l1() const {
  int height = 0;
  for (size_t i = 0; i < m_nodes.size(); i++) {
    int h                    = 0;
    PatchNode::ConstPtr node = m_nodes[i];

    if (node->get_type() != 2) {
      continue;
    }

    while (node->get_type() != 1) {
      node = node->get_parent();
      h++;
    }
    if (h > height) {
      height = h;
    }
  }
  return height;
}

//==============================================================================
int PatchTree::get_l2() const {
  int l2 = 0;
  for (size_t i = 0; i < m_nodes.size(); i++) {
    PatchNode::ConstPtr node = m_nodes[i];

    if (node->get_type() != 3) {
      continue;
    }

    while (node->get_type() != 2) {
      node = node->get_parent();
    }

    int num_children = node->get_children().size();
    if (num_children > l2)
      l2 = num_children;
  }

  // find smallest n such that 4^n > l2
  int n = 0;
  while (std::pow(4, n) < l2) {
    n++;
  }

  return n;
}

//==============================================================================
std::vector<int> PatchTree::get_leaves(int id) const {
  std::vector<int> leaves;
  std::vector<PatchNode::ConstPtr> pending{get_node(id)};
  while (!pending.empty()) {
    const auto node = pending.back();
    pending.pop_back();
    if (node->is_leaf()) {
      leaves.push_back(node->get_id());
      continue;
    }
    const auto children = node->get_children();
    for (auto child = children.rbegin(); child != children.rend(); ++child) {
      pending.push_back(*child);
    }
  }
  return leaves;
}
