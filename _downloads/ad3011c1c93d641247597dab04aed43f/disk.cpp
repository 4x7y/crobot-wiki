
#include "crobot/grasping/disk.hpp"

using namespace crobot;

//==============================================================================
Disk::Disk() {
  m_vertices.clear();
  m_plane_coords.clear();
  m_origin = math::VectorXd::Zero(6);
  m_t1     = math::VectorXd::Zero(6);
  m_t2     = math::VectorXd::Zero(6);
}

//==============================================================================
bool Disk::is_plane_construted() const {
  return m_t1.norm() > 0;
}

//==============================================================================
void Disk::consttruct_plane() {

  CROBOT_ASSERT(
      m_vertices.size() == 3,
      "At least 3 vertices are required to construct a plane");

  // std::cout << "plane is constructed" << std::endl;

  math::VectorXd v1 = m_vertices[0];
  math::VectorXd v2 = m_vertices[1];
  math::VectorXd v3 = m_vertices[2];

  math::VectorXd center = (v1 + v2 + v3) / 3.0;

  math::VectorXd t1 = v2 - v1;
  t1.normalize();

  math::VectorXd e13 = v3 - v1;
  math::VectorXd t2  = e13 - e13.dot(t1) * t1;
  t2.normalize();

  // store the local coordinate system
  m_origin = center;
  m_t1     = t1;
  m_t2     = t2;

  // store the plane
  m_plane_coords.clear();
  for (size_t i = 0; i < m_vertices.size(); i++) {
    m_plane_coords.push_back(get_plane_coord(m_vertices[i]));
  }
}

//==============================================================================
math::Vector2d Disk::get_plane_coord(const math::VectorXd& vertex) const {
  CROBOT_ASSERT(
      is_plane_construted(),
      "Plane is not constructed yet. Call construct_plane() first");

  math::VectorXd e = vertex - m_origin;
  double x         = e.dot(m_t1);
  double y         = e.dot(m_t2);

  return math::Vector2d(x, y);
}

//==============================================================================
std::vector<math::VectorXd> Disk::add_vertex(const math::VectorXd& vertex) {

  std::vector<math::VectorXd> ret;

  if (m_vertices.size() < 2) {
    m_vertices.push_back(vertex);
    return ret;
  }

  if (m_vertices.size() == 2) {
    m_vertices.push_back(vertex);
    consttruct_plane();
    return ret;
  }

  math::Vector2d plane_coord = get_plane_coord(vertex);
  if (plane_coord.norm() < 1e-6) {
    CROBOT_ASSERT(false, "Vertex is not on the plane");
  }

  int k_index    = 0;
  int k_m1_index = 0;
  int k_m2_index = 0;
  int k_p1_index = 0;
  int k_p2_index = 0;
  // check where to insert the new vertex
  bool inserted = false;
  for (size_t i = 0; i < m_plane_coords.size(); i++) {
    size_t j = (i + 1) % m_plane_coords.size();

    if (is_between(plane_coord, i, j)) {
      m_plane_coords.insert(m_plane_coords.begin() + j, plane_coord);
      m_vertices.insert(m_vertices.begin() + j, vertex);
      k_index    = j;
      k_m1_index = (j - 1 + m_plane_coords.size()) % m_plane_coords.size();
      k_m2_index = (j - 2 + m_plane_coords.size()) % m_plane_coords.size();
      k_p1_index = (j + 1) % m_plane_coords.size();
      k_p2_index = (j + 2) % m_plane_coords.size();
      inserted   = true;
      break;
    }
  }

  if (!inserted) {
    std::cout << "vertex to be inserted: " << plane_coord.transpose()
              << std::endl;
    print_plane_coords();

    CROBOT_ASSERT(
        false, "Vertex is not between any two vertices, should not happen!");
  }

  ret.push_back(m_vertices[k_m2_index]);
  ret.push_back(m_vertices[k_m1_index]);
  ret.push_back(m_vertices[k_index]);
  ret.push_back(m_vertices[k_p1_index]);
  ret.push_back(m_vertices[k_p2_index]);

  return ret;

  // CROBOT_ASSERT(
  //     inserted, "Vertex is not between any two vertices, should not
  //     happen!");
}
//==============================================================================
double cross(const math::Vector2d& v1, const math::Vector2d& v2) {
  return v1[0] * v2[1] - v1[1] * v2[0];
}

//==============================================================================
bool Disk::is_between(const math::Vector2d& test, int i, int j) const {

  math::Vector2d v1 = m_plane_coords[i];
  math::Vector2d v2 = m_plane_coords[j];

  double cross1 = cross(test, v1);
  double cross2 = cross(test, v2);

  CROBOT_ASSERT((cross1 * cross2) != 0, "Vertices are collinear");

  if (cross1 * cross2 > 0) {
    return false;
  }

  double cross3 = cross(v1, v2);
  double cross4 = cross(v1, test);

  if (cross3 * cross4 < 0) {
    return false;
  }

  return true;
}

//==============================================================================
void Disk::print_vertices() const {
  std::cout << "vertices on disk: " << std::endl;
  for (size_t i = 0; i < m_vertices.size(); i++) {
    std::cout << m_vertices[i].transpose() << std::endl;
  }
}

//==============================================================================
void Disk::print_plane_coords() const {
  std::cout << "plane coords: " << std::endl;
  for (size_t i = 0; i < m_plane_coords.size(); i++) {
    std::cout << m_plane_coords[i].transpose() << std::endl;
  }
}

//==============================================================================
int Disk::get_num_vertices() const {
  return m_vertices.size();
}

//==============================================================================
bool Disk::contains(const math::VectorXd& vertex) const {
  for (size_t i = 0; i < m_vertices.size(); i++) {
    if ((m_vertices[i] - vertex).norm() < 1e-6) {
      return true;
    }
  }

  return false;
}

//==============================================================================

bool Disk::is_on(const math::VectorXd& vertex) const {
  // CROBOT_ASSERT(
  //     is_plane_construted(),
  //     "Plane is not constructed yet. Call construct_plane() first");

  if (contains(vertex))
    return true;

  if (!is_plane_construted())
    return false;

  std::vector<math::VectorXd> vec
      = {m_vertices[0], m_vertices[1], m_vertices[2]};

  VectorSet::Ptr W = std::make_shared<VectorSet>(vec);
  W->add_vector(vertex);
  if (!W->is_linearly_independent())
    return true;

  return false;
}
//==============================================================================
std::vector<math::VectorXd> Disk::get_vertices() const {
  return m_vertices;
}

//==============================================================================
math::VectorXd Disk::generate_random_point() const {
  CROBOT_ASSERT(
      is_plane_construted(),
      "Plane is not constructed yet. Call construct_plane() first");

  double x = std::rand() / (double)RAND_MAX;
  double y = std::rand() / (double)RAND_MAX;

  math::VectorXd point = m_origin + x * m_t1 + y * m_t2;

  return point;
}
