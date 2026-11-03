#include "crobot/geometry/patch_mesh_data.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>

namespace crobot::geometry {
namespace {

template <typename T, size_t N>
std::vector<std::array<T, N>> read_csv(const std::string& filename) {
  std::ifstream input(filename);
  if (!input) {
    throw std::runtime_error("Cannot open patch mesh file: " + filename);
  }
  std::vector<std::array<T, N>> rows;
  std::string line;
  while (std::getline(input, line)) {
    std::istringstream row(line);
    std::array<T, N> values;
    for (size_t i = 0; i < N; ++i) {
      char delimiter;
      if (!(row >> values[i]) || !std::isfinite(static_cast<double>(values[i]))
          || (i + 1 < N && (!(row >> delimiter) || delimiter != ','))) {
        throw std::runtime_error(
            "Invalid CSV row " + std::to_string(rows.size() + 1) + " in "
            + filename);
      }
    }
    row >> std::ws;
    if (!row.eof()) {
      throw std::runtime_error("Extra CSV fields in " + filename);
    }
    rows.push_back(values);
  }
  if (!input.eof()) {
    throw std::runtime_error("Cannot read patch mesh file: " + filename);
  }
  return rows;
}

int global_label(
    int label,
    const PatchMeshData::Partition& partition,
    bool allow_boundary,
    const std::string& path) {
  if (label == -1 || (allow_boundary && label == 0)) {
    return label;
  }
  if (label <= 0 || label > partition.patch_count) {
    throw std::runtime_error("Invalid patch label in " + path);
  }
  return partition.first_patch_id + label - 1;
}

} // namespace

PatchMeshData::ConstPtr PatchMeshData::load(
    const std::string& folder, int num_partitions) {
  if (num_partitions < 0) {
    throw std::invalid_argument("Negative partition count");
  }
  if (num_partitions == 0) {
    return load(folder, std::vector<int>{});
  }
  const auto counts = read_csv<int, 1>(folder + "/patch_offsets.csv");
  if (counts.size() < static_cast<size_t>(num_partitions)) {
    throw std::runtime_error("Not enough patch offsets in " + folder);
  }
  std::vector<int> sizes;
  for (int i = 0; i < num_partitions; ++i) {
    sizes.push_back(counts[i][0]);
  }
  return load(folder, sizes);
}

PatchMeshData::ConstPtr PatchMeshData::load(
    const std::string& folder, const std::vector<int>& partition_sizes) {
  auto data           = std::shared_ptr<PatchMeshData>(new PatchMeshData);
  data->m_folder      = folder;
  data->m_num_patches = 1 + static_cast<int>(partition_sizes.size());
  for (size_t i = 0; i < partition_sizes.size(); ++i) {
    if (partition_sizes[i] < 0) {
      throw std::invalid_argument("Negative patch count");
    }
    const std::string path = folder + "/" + std::to_string(i) + "/";
    const auto vertices    = read_csv<double, 3>(path + "vertices.csv");
    auto faces             = read_csv<int, 3>(path + "faces.csv");
    const auto labels      = read_csv<int, 1>(path + "face_labels.csv");
    if (faces.size() != labels.size()) {
      throw std::runtime_error("Face and label counts differ in " + path);
    }
    const Partition partition{
        data->m_num_patches,
        partition_sizes[i],
        static_cast<int>(data->m_vertices.size()),
        static_cast<int>(vertices.size())};
    data->m_partitions.push_back(partition);
    data->m_num_patches += partition.patch_count;
    for (const auto& vertex : vertices) {
      data->m_vertices.emplace_back(vertex[0], vertex[1], vertex[2]);
    }
    for (size_t face = 0; face < faces.size(); ++face) {
      for (int& vid : faces[face]) {
        if (vid < 0 || vid >= partition.vertex_count) {
          throw std::runtime_error("Invalid face vertex index in " + path);
        }
        vid += partition.first_vertex_id;
      }
      data->m_faces.push_back(faces[face]);
      data->m_face_labels.push_back(
          global_label(labels[face][0], partition, false, path));
    }
  }
  return data;
}

const std::string& PatchMeshData::get_folder() const {
  return m_folder;
}

const std::vector<PatchMeshData::Partition>& PatchMeshData::get_partitions()
    const {
  return m_partitions;
}

const std::vector<Eigen::Vector3d>& PatchMeshData::get_vertices() const {
  return m_vertices;
}

int PatchMeshData::get_num_patches() const {
  return m_num_patches;
}

std::vector<PatchMeshData::Edge> PatchMeshData::get_vertex_edges() const {
  std::set<Edge> edges;
  for (const auto& face : m_faces) {
    for (size_t i = 0; i < 3; ++i) {
      if (face[i] != face[(i + 1) % 3]) {
        edges.insert(std::minmax(face[i], face[(i + 1) % 3]));
      }
    }
  }
  return {edges.begin(), edges.end()};
}

PatchMeshData::PatchAdjacency PatchMeshData::get_patch_adjacency() const {
  std::map<std::array<double, 3>, int> vertex_ids;
  std::vector<int> welded_vertices;
  for (const auto& vertex : m_vertices) {
    const std::array<double, 3> key{vertex[0], vertex[1], vertex[2]};
    welded_vertices.push_back(
        vertex_ids.emplace(key, vertex_ids.size()).first->second);
  }
  std::map<Edge, std::set<int>> edge_patches;
  std::set<int> active_patches;
  for (size_t i = 0; i < m_faces.size(); ++i) {
    const int label = m_face_labels[i];
    if (label == -1) {
      continue;
    }
    std::array<int, 3> face;
    for (size_t corner = 0; corner < 3; ++corner) {
      face[corner] = welded_vertices[m_faces[i][corner]];
    }
    if (face[0] == face[1] || face[1] == face[2] || face[2] == face[0]) {
      throw std::runtime_error("Degenerate face edge in " + m_folder);
    }
    active_patches.insert(label);
    for (size_t edge = 0; edge < 3; ++edge) {
      edge_patches[std::minmax(face[edge], face[(edge + 1) % 3])].insert(label);
    }
  }
  std::set<Edge> edges;
  for (const auto& entry : edge_patches) {
    // Include every incident pair on a non-manifold edge.
    for (auto first = entry.second.begin(); first != entry.second.end();
         ++first) {
      auto second = first;
      for (++second; second != entry.second.end(); ++second) {
        edges.emplace(*first, *second);
      }
    }
  }
  return {
      {active_patches.begin(), active_patches.end()},
      {edges.begin(), edges.end()}};
}

PatchMeshData::VertexAttributes PatchMeshData::load_vertex_attributes() const {
  VertexAttributes attributes;
  for (size_t i = 0; i < m_partitions.size(); ++i) {
    const auto& partition  = m_partitions[i];
    const std::string path = m_folder + "/" + std::to_string(i) + "/";
    const auto normals     = read_csv<double, 3>(path + "normals.csv");
    const auto labels      = read_csv<int, 1>(path + "ver_labels.csv");
    if (normals.size() != static_cast<size_t>(partition.vertex_count)
        || labels.size() != normals.size()) {
      throw std::runtime_error("Vertex attribute counts differ in " + path);
    }
    for (size_t vertex = 0; vertex < normals.size(); ++vertex) {
      const auto& n = normals[vertex];
      attributes.normals.push_back(
          Eigen::Vector3d(n[0], n[1], n[2]).normalized());
      attributes.patch_ids.push_back(
          global_label(labels[vertex][0], partition, true, path));
    }
  }
  return attributes;
}

} // namespace crobot::geometry
