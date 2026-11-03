
#include "crobot/grasping/convex_hull.hpp"

using namespace crobot;

ConvexHull::ConvexHull(
    VectorSet::Ptr vertex_set,
    std::vector<int> disk_index,
    int num_disks,
    VisualizationCallback visualization_callback) {
  m_visualization_callback = visualization_callback;
  m_facets.clear();
  m_normals.clear();
  m_dist2origin.clear();

  m_vertex_set = vertex_set;

  m_facets = vertex_set->get_subsets_down_1();
  for (size_t i = 0; i < m_facets.size(); i++) {
    auto ret = get_facet_normal_dist(m_facets[i]);
    m_normals.push_back(ret.first);
    m_dist2origin.push_back(ret.second);
  }

  // test if it contains the origin
  // for (size_t i = 0; i < m_facets.size(); i++) {
  //   VectorSet::Ptr f_left = vertex_set->substract(m_facets[i]);
  //   if (m_normals[i].dot(f_left->get_vectors()[0]) > 0) {

  //     print_facet(m_facets[i], "facet" + std::to_string(i));
  //     std::cout << f_left->get_vectors()[0].transpose() << std::endl;
  //     CROBOT_ASSERT(
  //         false,
  //         "The convex hull does not contain the origin, should not happen");
  //   }
  // }

  for (int i = 0; i < num_disks; i++) {
    m_disks.push_back(std::make_shared<Disk>());
  }

  // initialize disks
  // std::cout << "disk_index.size() = " << disk_index.size() << std::endl;
  for (size_t i = 0; i < disk_index.size(); i++) {
    m_disks[disk_index[i]]->add_vertex(vertex_set->get_vectors()[i]);
  }

  m_special_inner = vertex_set->get_vectors().back();
}

//==============================================================================
ConvexHull::ConvexHull(
    VectorSet::Ptr vertex_set, std::vector<int> disk_index, int num_disks) {
  m_facets.clear();
  m_normals.clear();
  m_dist2origin.clear();

  m_vertex_set = vertex_set;

  m_facets = vertex_set->get_subsets_down_1();
  for (size_t i = 0; i < m_facets.size(); i++) {
    auto ret = get_facet_normal_dist(m_facets[i]);
    m_normals.push_back(ret.first);
    m_dist2origin.push_back(ret.second);
  }

  // test if it contains the origin
  // for (size_t i = 0; i < m_facets.size(); i++) {
  //   VectorSet::Ptr f_left = vertex_set->substract(m_facets[i]);
  //   if (m_normals[i].dot(f_left->get_vectors()[0]) > 0) {

  //     print_facet(m_facets[i], "facet" + std::to_string(i));
  //     std::cout << f_left->get_vectors()[0].transpose() << std::endl;
  //     CROBOT_ASSERT(
  //         false,
  //         "The convex hull does not contain the origin, should not happen");
  //   }
  // }

  for (int i = 0; i < num_disks; i++) {
    m_disks.push_back(std::make_shared<Disk>());
  }

  // initialize disks
  for (size_t i = 0; i < disk_index.size(); i++) {
    m_disks[disk_index[i]]->add_vertex(vertex_set->get_vectors()[i]);
  }

  m_special_inner = vertex_set->get_vectors().back();
}

//==============================================================================
std::pair<math::VectorXd, double> ConvexHull::get_facet_normal_dist(
    VectorSet::Ptr facet) const {

  math::VectorXd vec_ref = facet->get_vectors()[facet->get_size() - 1];
  math::VectorXd normal  = facet
                              ->pwise_substract(vec_ref)
                              // ->get_orthogonal_vector()
                              ->get_orthogonal_vector_by_GS(vec_ref)
                              .normalized();
  double dist2origin = normal.dot(vec_ref);

  // for (size_t i = 1; i < facet->get_vectors().size(); i++) {
  //   double dist = normal.dot(facet->get_vectors()[i]);
  //   CROBOT_ASSERT(
  //       std::abs(dist - dist2origin) < 1e-6,
  //       "dist = %f, dist2origin = %f, should be the same",
  //       dist,
  //       dist2origin);
  // }

  // math::VectorXd normal2
  //     =
  //     facet->pwise_substract(vec_ref)->get_orthogonal_vector().normalized();
  // //->get_orthogonal_vector_by_GS(vec_ref);
  // double dist2origin2 = normal2.dot(vec_ref);

  // if (dist2origin2 < 0) {
  //   normal2 *= -1;
  //   dist2origin2 *= -1;
  // }

  // CROBOT_ASSERT(
  //     std::abs(dist2origin2 - dist2origin) < 1e-6,
  //     "dist2origin2 = %f, dist2origin = %f, should be the same",
  //     dist2origin2,
  //     dist2origin);

  // CROBOT_ASSERT(normal.isApprox(normal2, 1e-6), "normal is not the same");

  CROBOT_ASSERT(
      dist2origin > 0, "dist2origin = %f < 0, should not happen", dist2origin);

  return std::make_pair(normal, dist2origin);
}

//==============================================================================
int ConvexHull::get_min_index(double lb) const {
  int min_index = 0;
  for (size_t i = 1; i < m_dist2origin.size(); i++) {
    if (m_dist2origin[i] > lb && m_dist2origin[i] < m_dist2origin[min_index]) {
      min_index = i;
    }
  }
  // CROBOT_ASSERT(m_dist2origin[min_index] >= lb, "No minimum index found");
  if (m_dist2origin[min_index] <= lb) {
    return -1;
  }
  return min_index;
}

//==============================================================================
double ConvexHull::get_dist2origin(int index) const {
  return m_dist2origin[index];
}

//==============================================================================
math::VectorXd ConvexHull::get_normal(int index) const {
  return m_normals[index];
}

//==============================================================================
void ConvexHull::add_vertex(
    const math::VectorXd& new_v,
    int disk_index,
    int facet_index [[maybe_unused]]) {

  CROBOT_ASSERT(
      !m_disks[disk_index]->contains(new_v),
      "The new vertex is already in the convex hull");

  m_vertex_set->add_vector(new_v);

  auto ret = m_disks[disk_index]->add_vertex(new_v);
  (void)ret; // Unused variable

  (void)get_dist2origin(get_min_index(-1)); // Result not used

  std::vector<bool> eliminate(m_facets.size(), false);

  std::vector<VectorSet::Ptr> disappearing_facets;
  for (size_t i = 0; i < m_facets.size(); i++) {
    if (m_normals[i].dot(new_v) > m_dist2origin[i]) {
      disappearing_facets.push_back(m_facets[i]);
      remove_facet(i);
      i--;
    }
  }

  std::vector<VectorSet::Ptr> new_facets;

  // check every 4d facet (edge with 5 vectors) of the disapearing facets
  for (size_t i = 0; i < disappearing_facets.size(); i++) {
    // generate all edges
    std::vector<VectorSet::Ptr> edges
        = disappearing_facets[i]->get_subsets_down_1();

    for (size_t j = 0; j < edges.size(); j++) {

      edges[j]->add_vector(new_v);

      if (contains_facet(m_facets, edges[j])
          || contains_facet(new_facets, edges[j])
          || !edges[j]->is_linearly_independent()
          || !is_facet_on_CH(edges[j])) {
        continue;
      }

      // auto dn_before = get_facet_normal_dist(disappearing_facets[i]);
      // auto dn_after = get_facet_normal_dist(edges[j]);

      // if (dn_after.second < dist_lb - 1e-6) { // &&
      //   print_facet(edges[j], "[ERROR] ridge from " + std::to_string(i));
      //   std::cout << std::endl;

      //   CROBOT_ASSERT(
      //       false,
      //       "dist_after = %f < closest_dist before = %f, should not happen",
      //       dn_after.second,
      //       dist_lb);
      //   continue;
      // }
      new_facets.push_back(edges[j]);
    }
  } // end of for disappearing_facets

  // add new vertex to the disks
  std::vector<VectorSet::Ptr> new_facets_2;
  std::vector<VectorSet::Ptr> new_facets_3;

  if (ret.size() > 0) { //

    // [k_m2_index]; 0
    // [k_m1_index]; 1
    // [k_index];    2
    // [k_p1_index]; 3
    // [k_p2_index]; 4

    // create disp + new facets for 1
    std::vector<math::VectorXd> disp_1_vec       = {ret[0], ret[1], ret[3]};
    std::vector<math::VectorXd> new_facet_11_vec = {ret[0], ret[1], ret[2]};
    std::vector<math::VectorXd> new_facet_12_vec = {ret[1], ret[2], ret[3]};
    VectorSet::Ptr disp_1       = std::make_shared<VectorSet>(disp_1_vec);
    VectorSet::Ptr new_facet_11 = std::make_shared<VectorSet>(new_facet_11_vec);
    VectorSet::Ptr new_facet_12 = std::make_shared<VectorSet>(new_facet_12_vec);

    // create disp + new facets for 2
    std::vector<math::VectorXd> disp_2_vec       = {ret[1], ret[3], ret[4]};
    std::vector<math::VectorXd> new_facet_21_vec = {ret[1], ret[2], ret[3]};
    std::vector<math::VectorXd> new_facet_22_vec = {ret[2], ret[3], ret[4]};
    VectorSet::Ptr disp_2       = std::make_shared<VectorSet>(disp_2_vec);
    VectorSet::Ptr new_facet_21 = std::make_shared<VectorSet>(new_facet_21_vec);
    VectorSet::Ptr new_facet_22 = std::make_shared<VectorSet>(new_facet_22_vec);

    bool delete_disp = !disp_1->is_identical(disp_2); // delete if not identical
    // std::cout << "delete_disp = " << delete_disp << std::endl;
    // go through all facets to replace triangles
    for (size_t i = 0; i < m_facets.size(); i++) {
      VectorSet::Ptr facet = m_facets[i];
      bool update          = false;
      if (facet->contains(disp_1)) {
        update               = true;
        VectorSet::Ptr new_1 = facet->replaced(disp_1, new_facet_11);
        VectorSet::Ptr new_2 = facet->replaced(disp_1, new_facet_12);
        if (!contains_facet(new_facets_2, new_1))
          new_facets_2.push_back(new_1);
        if (!contains_facet(new_facets_2, new_2))
          new_facets_2.push_back(new_2);
      }

      if (facet->contains(disp_2)) {
        update               = true;
        VectorSet::Ptr new_1 = facet->replaced(disp_2, new_facet_21);
        VectorSet::Ptr new_2 = facet->replaced(disp_2, new_facet_22);

        if (!contains_facet(new_facets_2, new_1))
          new_facets_2.push_back(new_1);
        if (!contains_facet(new_facets_2, new_2))
          new_facets_2.push_back(new_2);
      }

      if (update && delete_disp) {
        remove_facet(i);
        i--;
      }
    }

    // create disp edge
    std::vector<math::VectorXd> disp_edge_vec = {ret[1], ret[3]};
    VectorSet::Ptr disp_edge = std::make_shared<VectorSet>(disp_edge_vec);
    std::vector<math::VectorXd> disp_edge_1_vec = {ret[1], ret[2]};
    std::vector<math::VectorXd> disp_edge_2_vec = {ret[2], ret[3]};
    VectorSet::Ptr disp_edge_1 = std::make_shared<VectorSet>(disp_edge_1_vec);
    VectorSet::Ptr disp_edge_2 = std::make_shared<VectorSet>(disp_edge_2_vec);

    // go through all facets to replace disappearing edge
    for (size_t i = 0; i < m_facets.size(); i++) {
      VectorSet::Ptr facet = m_facets[i];
      if (facet->contains(disp_edge)) {

        VectorSet::Ptr new_1 = facet->replaced(disp_edge, disp_edge_1);
        VectorSet::Ptr new_2 = facet->replaced(disp_edge, disp_edge_2);

        if (!contains_facet(new_facets_2, new_1))
          new_facets_3.push_back(new_1);

        if (!contains_facet(new_facets_2, new_2))
          new_facets_3.push_back(new_2);

        remove_facet(i);
        i--;
      }
    }
  } // end for updating facets

  // add new facets to m_facets
  for (size_t i = 0; i < new_facets.size(); i++) {
    add_facet(new_facets[i], "");
    //   if (m_dist2origin.back() < dist_lb - 1e-6) {
    //     // remove_facet(m_facets.size() - 1);
    //     // continue;
    //     CROBOT_ASSERT(
    //         false,
    //         "dist2origin = %f < dist_lb = %f, due to ridge",
    //         m_dist2origin.back(),
    //         dist_lb);
    //   }
  }

  for (size_t i = 0; i < new_facets_2.size(); i++) {
    add_facet(new_facets_2[i], "");
    // if (m_dist2origin.back() < dist_lb - 1e-6) {
    //   // remove_facet(m_facets.size() - 1);
    //   // continue;
    //   CROBOT_ASSERT(
    //       false,
    //       "dist2origin = %f < dist_lb = %f, due to plane from 2",
    //       m_dist2origin.back(),
    //       dist_lb);
    // }
  }

  for (size_t i = 0; i < new_facets_3.size(); i++) {
    add_facet(new_facets_3[i], "");
    // if (m_dist2origin.back() < dist_lb - 1e-6) {
    //   // remove_facet(m_facets.size() - 1);
    //   // continue;
    //   CROBOT_ASSERT(
    //       false,
    //       "dist2origin = %f < dist_lb = %f, due to plane from 3",
    //       m_dist2origin.back(),
    //       dist_lb);
    // }
  }

  // std::cout << "disappearing facets: " << disappearing_facets.size()
  //           << std::endl;
  // std::cout << "new number of facets: " << m_facets.size() << std::endl;

  // plot_facets(new_v);
}

//==============================================================================
double ConvexHull::get_max_index() const {
  double max_dist = 0;
  int max_index   = 0;
  for (size_t i = 0; i < m_dist2origin.size(); i++) {
    if (m_dist2origin[i] > max_dist) {
      max_dist  = m_dist2origin[i];
      max_index = i;
    }
  }
  return max_index;
}

//==============================================================================
bool ConvexHull::is_empty() const {
  return m_facets.size() == 0;
}

//==============================================================================
bool ConvexHull::contains_facet(
    const std::vector<VectorSet::Ptr>& facets, const VectorSet::Ptr f) const {
  for (size_t i = 0; i < facets.size(); i++) {
    if (facets[i]->is_identical(f)) {
      return true;
    }
  }
  return false;
}

//==============================================================================

bool ConvexHull::contains_vertex(const math::VectorXd& vertex) const {
  for (size_t i = 0; i < m_facets.size(); i++) {
    if (m_facets[i]->contains(vertex, 1e-6)) {
      return true;
    }
  }
  return false;
}

//==============================================================================
void ConvexHull::remove_facet(int index) {
  m_facets.erase(m_facets.begin() + index);
  m_normals.erase(m_normals.begin() + index);
  m_dist2origin.erase(m_dist2origin.begin() + index);
}

//==============================================================================
void ConvexHull::add_facet(VectorSet::Ptr facet, const std::string& add_msg) {

  if (add_msg != "") {
    std::cout << add_msg << std::endl;

    // print facet
    std::cout << facet->get_stacked_vectors() << std::endl;
  }

  auto ret = get_facet_normal_dist(facet);
  m_facets.push_back(facet);
  m_normals.push_back(ret.first);
  m_dist2origin.push_back(ret.second);
}

//==============================================================================
// bool ConvexHull::is_shared_edge(
//     const VectorSet::Ptr edge, // 5 vertices
//     const VectorSet::Ptr f1,   // 6 vertices
//     const VectorSet::Ptr f2) const {

//   if (f1->contains(edge) && f2->contains(edge)) {
//     return true;
//   }

//   // check for points on the same plane
//   for (size_t i = 0; i < m_disks.size(); i++) {
//     VectorSet::ConstPtr d
//         = std::make_shared<const VectorSet>(m_disks[i]->get_vertices());

//     if (d->get_size() < 3)
//       continue; // not enough points to form a plane

//     VectorSet::Ptr f1_d   = f1->substract(d);
//     VectorSet::Ptr f2_d   = f2->substract(d);
//     VectorSet::Ptr edge_d = edge->substract(d);

//     if (f1_d->get_size() == 3 && f2_d->get_size() == 3
//         && edge_d->get_size() == 2) {
//       if (f1_d->contains(edge_d) && f2_d->contains(edge_d)) {
//         std::cout << "found shared edge" << std::endl;
//         return true;
//       }
//     }
//   }

//   return false;
// }

//==============================================================================
void ConvexHull::print_facet(
    const VectorSet::Ptr facet, const std::string& name) const {

  // print name in red
  std::cout << "\033[1;36m";
  std::cout << name << std::endl;
  // stop red
  std::cout << "\033[0m";
  std::cout << facet->get_stacked_vectors().row(0) << std::endl; // .row(0)

  for (int i = 0; i < facet->get_size(); i++) {
    bool found = false;
    for (size_t j = 0; j < m_disks.size(); j++) {
      if (m_disks[j]->contains(facet->get_vectors()[i])) {
        found = true;
        std::cout << j << " ";
      }
    }
    if (!found)
      std::cout << "x ";
  }
  std::cout << std::endl;
}

//==============================================================================
bool ConvexHull::is_facet_on_CH(const VectorSet::Ptr facet) const {
  // check if all other vertices are on the other side
  auto dn = get_facet_normal_dist(facet);
  VectorSet::Ptr other_vertics
      = m_vertex_set->substract(facet)->pwise_substract(
          facet->get_vectors()[0]);
  for (int k = 0; k < other_vertics->get_size(); k++) {
    if (dn.first.dot(other_vertics->get_vectors()[k]) > 1e-7) {
      return false;
    }
  }
  return true;
}

//==============================================================================
void ConvexHull::plot_facets(const math::VectorXd& point) const {

  if (m_visualization_callback) {
    std::vector<std::pair<math::Vector3d, math::Vector3d>> lines;
    std::vector<math::Vector3d> points;

    points.push_back(point);
    points.push_back(math::Vector3d::Zero());

    std::cout << "start_ploting " << m_facets.size() << std::endl;
    CROBOT_ASSERT(m_facets[0]->get_vectors()[0].size() == 3, "only support 3D");
    for (size_t i = 0; i < m_facets.size(); i++) {
      std::vector<VectorSet::Ptr> edges = m_facets[i]->get_subsets_down_1();
      for (size_t j = 0; j < edges.size(); j++) {
        lines.push_back(std::make_pair(
            edges[j]->get_vectors()[0], edges[j]->get_vectors()[1]));
      }
    }

    m_visualization_callback(lines, points);
  }
}

//==============================================================================
bool ConvexHull::exceeds(int num_facets) const {
  return static_cast<int>(m_facets.size()) > num_facets;
}
