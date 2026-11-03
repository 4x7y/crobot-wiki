
#include "crobot/grasping/grasp_solver.hpp"

#include <queue>
#include "crobot/grasping/contact_filter.hpp"

#include "yaml-cpp/yaml.h"

using namespace crobot;

//==============================================================================
GraspSolver::GraspSolver(
    geometry::MeshPatches::ConstPtr mesh,
    GraspQuality::ConstPtr quality,
    crobot::patch_tree::PatchTree::ConstPtr patch_tree)
  : m_mesh(mesh), m_quality(quality), m_patch_tree(patch_tree) {
}

//==============================================================================
GraspSolver::GraspSolver(
    geometry::MeshPatches::ConstPtr mesh,
    GraspQuality::ConstPtr quality,
    crobot::patch_tree::PatchTree::ConstPtr patch_tree,
    VisualizationCallback visualization_callback)
  : m_mesh(mesh),
    m_quality(quality),
    m_patch_tree(patch_tree),
    m_visualization_callback(visualization_callback) {
}
//==============================================================================
// use patch tree
std::pair<double, std::vector<int>> GraspSolver::solve_grasp_BFS(
    int num_fingers) {
  m_num_finger_adj = 0;

  std::priority_queue<
      std::pair<double, std::vector<int>>,
      std::vector<std::pair<double, std::vector<int>>>,
      GraspSolver::Compare>
      pq; // overall

  std::priority_queue<
      std::pair<double, std::vector<int>>,
      std::vector<std::pair<double, std::vector<int>>>,
      GraspSolver::Compare>
      pq2; // quality < 0.22

  std::priority_queue<
      std::pair<double, std::vector<int>>,
      std::vector<std::pair<double, std::vector<int>>>,
      GraspSolver::Compare>
      pq1; // quality < 0.12

  std::priority_queue<
      std::pair<double, std::vector<int>>,
      std::vector<std::pair<double, std::vector<int>>>,
      GraspSolver::Compare>
      pq0; // quality < 0.02

  std::vector<int> child_ids = m_patch_tree->get_child_ids(0);

  std::vector<std::vector<int>> subsets
      = generate_permutations_with_duplicates(child_ids, num_fingers);

  std::cout << "initial number of combinations: " << subsets.size()
            << std::endl;

  std::cin.ignore();

  std::vector<std::pair<int, std::vector<int>>> comb_queue;

  for (auto& subset : subsets) {
    if (m_patch_tree->is_valid_combination(subset))
      comb_queue.push_back(std::make_pair(0, subset));
  }

  int counter_exam = 0;
  while (comb_queue.size() > 0) {
    // std::cout << "Remaining " << comb_queue.size() << " combinations"
    //           << std::endl;
    auto comb = comb_queue.back();
    comb_queue.pop_back();

    std::vector<std::vector<int>> next_expand_comb
        = m_patch_tree->get_next_expand_combination(comb.second, comb.first);

    if (next_expand_comb.size() == 0) {

      // if (comb == std::make_pair(0, std::vector<int>{69, 75, 86})) {
      //   std::cout << "checked!!!!!!!!!!!!" << std::endl;
      //   std::cin.ignore();
      // }

      if (!m_mesh->is_center_defined(comb.second)) {
        continue;
      }

      if (!grasping::are_patch_centers_nearby(*m_mesh, comb.second)) {
        continue;
      }

      // all leaves, and satisfy the distance check for the centers,
      // do the grasp quality measure
      double ret = math::Vector1d::Random()[0];
      // double ret = get_center_grasp_quality(comb.second);

      // if (comb == std::make_pair(0, std::vector<int>{69, 75, 86})) {
      //   std::cout << "ret = " << ret << std::endl;
      // }

      std::vector<int> test_vertex_indices;
      for (int label : comb.second) {
        test_vertex_indices.push_back(
            m_mesh->get_patch_center(label)->get_id());
      }

      // overall rank
      pq.push(std::make_pair(ret, comb.second));

      // keep top 100 in the queue
      if (pq.size() > 20) {
        // remove the smallest quality
        pq.pop();
      }

      // rank <0.02
      if (ret < 0.05) {
        pq0.push(std::make_pair(ret, test_vertex_indices));
        if (pq0.size() > 20) {
          pq0.pop();
        }
      }
      // rank <0.12
      else if (ret < 0.12) {
        pq1.push(std::make_pair(ret, test_vertex_indices));
        if (pq1.size() > 20) {
          pq1.pop();
        }
      }
      // rank <0.22
      else if (ret < 0.20) {
        pq2.push(std::make_pair(ret, test_vertex_indices));
        if (pq2.size() > 20) {
          pq2.pop();
        }
      }

      counter_exam++;

      if (counter_exam % 1000 == 0)
        std::cout << "tested " << counter_exam << " combinations" << std::endl;

      continue;
    }

    // add the next level combinations to the queue
    for (size_t i = 0; i < next_expand_comb.size(); i++) {
      int next_id                = (comb.first + 1) % num_fingers;
      std::vector<int> next_comb = next_expand_comb[i];

      if (!m_patch_tree->is_checkable(next_comb)
          || m_patch_tree->is_valid(next_comb)) {
        comb_queue.push_back(std::make_pair(next_id, next_comb));
        // std::cout << "pushing " << next_id << std::endl;
      }
      // else
      //   std::cout << "invalid combination" << std::endl;
    }
  }

  std::cout << "examined " << counter_exam << " combinations" << std::endl;

  std::priority_queue<
      std::pair<double, std::vector<int>>,
      std::vector<std::pair<double, std::vector<int>>>,
      GraspSolver::Compare>
      result_pq;

  // go through the queue
  while (!pq.empty()) {
    std::cout << "-------" << std::endl;
    std::cout << "working on " << pq.size() << " combinations" << std::endl;
    std::cout << "comb = ";
    for (int index : pq.top().second) {
      std::cout << index << " ";
    }
    std::cout << std::endl;
    std::cout << "initial quality: " << pq.top().first << std::endl;
    std::vector<int> comb = pq.top().second;
    pq.pop();

    int current_num_adj = m_num_finger_adj;
    auto ret            = solve_grasp(comb, 20, -1);
    std::cout << "improved to " << ret.first << " in "
              << (m_num_finger_adj - current_num_adj) << " adjustments"
              << std::endl;

    update_sim(ret.second);

    result_pq.push(std::make_pair(ret.first, ret.second));
  }

  // save_yaml(m_mesh->get_folder() + "/result_all.yaml", result_pq);
  // save_yaml(m_mesh->get_folder() + "/result_2.yaml", pq2);
  // save_yaml(m_mesh->get_folder() + "/result_1.yaml", pq1);
  // save_yaml(m_mesh->get_folder() + "/result_0.yaml", pq0);

  return std::make_pair(0, std::vector<int>());
}

//==============================================================================
std::pair<double, std::vector<int>> GraspSolver::solve_grasp(int num_fingers) {
  CROBOT_ASSERT(m_patch_tree != nullptr);

  m_num_finger_adj = 0;

  std::priority_queue<
      std::pair<double, std::vector<int>>,
      std::vector<std::pair<double, std::vector<int>>>,
      GraspSolver::Compare>
      pq;

  std::vector<int> child_ids = m_patch_tree->get_child_ids(0);

  std::cout << "\n\nchildren of root: " << child_ids.size() << std::endl;

  std::vector<std::vector<int>> comb_queue
      = generate_permutations_with_duplicates(child_ids, num_fingers);

  // for (size_t i = 0; i < comb_queue.size(); i++) {
  //   std::vector<int> comb = comb_queue[i];
  //   if (m_patch_tree->is_checkable(comb) && !m_patch_tree->is_valid(comb)) {
  //     comb_queue.erase(comb_queue.begin() + i);
  //     i--;
  //   }
  // }

  std::cout << "initial number of combinations: " << comb_queue.size()
            << std::endl;

  std::cin.ignore();

  int counter_exam = 0;

  while (comb_queue.size() > 0) {

    // randomize the order of the combinations
    // if (counter_exam % 500 == 0)
    //   std::random_shuffle(comb_queue.begin(), comb_queue.end());

    // if (counter_exam > 30000) {
    //   break;
    // }

    std::vector<int> comb = comb_queue.back();
    comb_queue.pop_back();

    std::vector<std::vector<int>> next_level_comb
        = m_patch_tree->get_next_level_combination(comb);

    if (next_level_comb.size() == 0) { // all leaves

      // std::cout << "examining: ";
      // for (int index : comb) {
      //   std::cout << index << " ";
      // }
      // std::cout << std::endl;

      // if (counter < 200) {

      // if (!m_mesh->is_center_defined(comb)) {
      //   continue;
      // }

      double ret = get_center_grasp_quality(comb);
      // double ret = -1;
      pq.push(std::make_pair(ret, comb));

      // keep top 100 in the queue
      if (pq.size() > 20) {
        // remove the smallest quality
        pq.pop();
      }
      counter_exam++;

      if (counter_exam % 10000 == 0)
        std::cout << "tested " << counter_exam << " combinations" << std::endl;

      continue;
    }

    for (size_t i = 0; i < next_level_comb.size(); i++) {
      std::vector<int> next_comb = next_level_comb[i];
      if (!m_patch_tree->is_checkable(next_comb)
          || m_patch_tree->is_valid(next_comb)) {
        comb_queue.push_back(next_comb);
      }
    }
  }

  double max_quality = -9999;
  std::vector<int> best_vertices;
  std::vector<int> best_comb;

  // go through the queue
  while (!pq.empty()) {
    std::cout << "-------" << std::endl;
    std::cout << "working on " << pq.size() << " combinations" << std::endl;
    std::cout << "comb = ";
    for (int index : pq.top().second) {
      std::cout << index << " ";
    }
    std::cout << std::endl;
    std::cout << "initial quality: " << pq.top().first << std::endl;
    std::vector<int> comb = pq.top().second;
    pq.pop();

    int current_num_adj = m_num_finger_adj;
    auto ret            = solve_grasp(comb, 50, -1);
    std::cout << "improved to " << ret.first << " in "
              << (m_num_finger_adj - current_num_adj) << " adjustments"
              << std::endl;

    if (ret.first > max_quality) {
      max_quality   = ret.first;
      best_vertices = ret.second;
      best_comb     = comb;
      // std::cout << "1" << std::endl;
      std::cout << "[IMPROVED] " << max_quality << " with comb ";
      for (int index : best_comb) {
        std::cout << index << " ";
      }
      std::cout << std::endl;

      update_sim(best_vertices);
      //}
    }
  }

  std::cout << "\n\nsolved! " << std::endl;
  std::cout << "examined " << counter_exam << " combinations" << std::endl;
  std::cout << "max_quality: " << max_quality << std::endl;
  std::cout << "best_vertices: ";
  for (int index : best_vertices) {
    std::cout << index << " ";
  }
  std::cout << std::endl;
  std::cout << "best_comb: ";
  for (int index : best_comb) {
    std::cout << index << " ";
  }
  std::cout << std::endl;

  std::cout << "total number of finger adjs: " << m_num_finger_adj << std::endl;

  return std::make_pair(max_quality, best_vertices);
}

//==============================================================================
std::pair<double, std::vector<int>> GraspSolver::solve_grasp_naive(
    int num_fingers) {
  int num_patches = m_mesh->get_num_patches();
  CROBOT_ASSERT(num_fingers <= num_patches);

  m_num_finger_adj = 0;

  std::vector<int> patch_labels;
  for (int i = 0; i < num_patches; i++) {
    if (m_mesh->get_patch_center(i) != nullptr) // patch exists
      patch_labels.push_back(i);
  }

  std::cout << "number of patches: " << patch_labels.size() << std::endl;

  // generate all possible k combinations of patch_labels
  std::vector<std::vector<int>> combinations
      = generateSubsets(patch_labels, num_fingers, 0);

  std::cout << "number of combinations: " << combinations.size() << std::endl;

  double max_quality = 0.0;
  std::vector<int> best_vertex_indices;

  int counter = 0;

  for (auto& combination : combinations) {
    counter++;
    if (counter % (combinations.size() / 100) == 0) {
      std::cout << "\033[1;31m";
      std::cout << "progress: " << (double)counter * 100 / combinations.size()
                << "%" << std::endl;
      std::cout << "\033[0m";
    }

    std::vector<int> test_vertex_indices;
    for (int label : combination) {
      test_vertex_indices.push_back(m_mesh->get_patch_center(label)->get_id());
    }

    // std::cout << "======" << std::endl;

    auto ret = solve_grasp(combination, 50, max_quality - 0.05);
    if (ret.first < 0.0) {
      continue;
    }

    if (ret.first > max_quality) {
      std::cout << "improved with combination: ";
      for (int label : combination) {
        std::cout << label << " ";
      }
      std::cout << std::endl;
      max_quality         = ret.first;
      best_vertex_indices = ret.second;
      update_sim(best_vertex_indices);
      std::cout << "quality: " << max_quality << std::endl;
    }
  }

  std::cout << "max_quality: " << max_quality << std::endl;
  // update_sim(best_vertex_indices);
  return std::make_pair(max_quality, best_vertex_indices);
}

//==============================================================================
double GraspSolver::get_center_grasp_quality(
    const std::vector<int>& patch_labels) const {
  std::vector<int> vertex_indices;
  for (int label : patch_labels) {
    if (m_mesh->get_patch_center(label) == nullptr)
      return -9999;
    vertex_indices.push_back(m_mesh->get_patch_center(label)->get_id());
  }
  return m_quality->get_grasp_quality(vertex_indices);
}

//==============================================================================
std::pair<double, std::vector<int>> GraspSolver::solve_grasp(
    const std::vector<int>& patch_labels, int max_round, double quality_lb) {

  for (size_t i = 0; i < patch_labels.size(); i++)
    if (m_mesh->get_patch_center(patch_labels[i]) == nullptr)
      return std::make_pair(-9999, std::vector<int>());
    else
      std::cout << m_mesh->get_patch_center(patch_labels[i])->get_id()
                << std::endl;

  std::vector<int> best_vertex_indices;
  std::vector<int> test_vertex_indices;

  // initialize best_vertex_indices to the centers of the patches
  for (int label : patch_labels) {
    best_vertex_indices.push_back(m_mesh->get_patch_center(label)->get_id());
  }

  double max_quality = m_quality->get_grasp_quality(best_vertex_indices);
  std::cout << "initial quality: " << max_quality << std::endl;

  if (max_quality < quality_lb) {
    return std::make_pair(max_quality, best_vertex_indices);
  }

  // std::cout << "initial quality: " << max_quality << std::endl;
  // std::cout << "initial vertex indices: ";
  // for (int index : best_vertex_indices) {
  //   std::cout << index << " ";
  // }
  // std::cout << std::endl;

  for (int i = 0; i < max_round; i++) {
    std::cout << "round " << i;

    // std::cout << " best vertex indices: ";
    // for (int index : best_vertex_indices) {
    //   std::cout << index << " ";
    // }
    // std::cout << std::endl;

    // std::cout << "vertex coordinates: \n";
    // for (int index : best_vertex_indices) {
    //   std::cout << m_mesh->get_vertex(index)->get_pos().transpose()
    //             << std::endl;
    // }
    // std::cout << std::endl;

    bool improved = false;

    // generate random processing permutation
    std::vector<int> process_order
        = generate_random_permutation(patch_labels.size());

    test_vertex_indices = best_vertex_indices;

    // go through each patch and try to improve the grasp
    // in the order of process_order
    for (int improve_index : process_order) {

      int vertex_index = test_vertex_indices[improve_index];

      std::vector<int> neighbor_ids
          = m_mesh->get_vertex(vertex_index)
                ->get_neighbor_ids_within_same_patch();

      neighbor_ids = permu(neighbor_ids);

      for (int neighbor : neighbor_ids) {
        test_vertex_indices[improve_index] = neighbor;

        // update_sim(test_vertex_indices);

        // std::cout << "testing: ";
        // for (size_t i = 0; i < test_vertex_indices.size(); i++) {
        //   std::cout << test_vertex_indices[i] << " ";
        // }
        // std::cout << std::endl;

        double quality = m_quality->get_grasp_quality(test_vertex_indices);

        if (quality > max_quality) {
          best_vertex_indices = test_vertex_indices;
          max_quality         = quality;
          improved            = true;

          // break; // ???????????????????
          // update visualization
          // update_sim(best_vertex_indices);
          // std::cout << "quality: " << max_quality << std::endl;
          // std::cout << "Press any key to continue" << std::endl;
          // std::cin.ignore();
        }
      }
    }

    if (!improved) {
      break;
    }

    std::cout << "improved vertices: \n";
    for (int index : best_vertex_indices) {
      math::Vector3d pos = m_mesh->get_vertex(index)->get_pos();
      std::cout << "[" << pos[0] << ", " << pos[1] << ", " << pos[2] << "],"
                << std::endl;
    }

    std::cout << "improved quality: " << max_quality << std::endl;
    std::cout << std::endl;

    m_num_finger_adj++;
  }

  std::cout << "final quality: " << max_quality << std::endl;

  return std::make_pair(max_quality, best_vertex_indices);
}

//==============================================================================
std::vector<int> GraspSolver::generate_random_permutation(int n) const {
  std::vector<int> permutation;
  for (int i = 0; i < n; i++) {
    permutation.push_back(i);
  }
  std::random_shuffle(permutation.begin(), permutation.end());
  return permutation;
}

//==============================================================================
void GraspSolver::update_sim(const std::vector<int>& vertex_indices) const {
  if (m_visualization_callback) {
    std::vector<math::Vector3d> vertex_positions;
    vertex_positions.reserve(vertex_indices.size());

    for (int index : vertex_indices) {
      vertex_positions.push_back(m_mesh->get_vertex(index)->get_pos());
    }

    m_visualization_callback(vertex_positions);
  }
}

//==============================================================================
std::vector<std::vector<int>> GraspSolver::generateSubsets(
    const std::vector<int>& set, int k, int index) const {
  std::vector<std::vector<int>> subsets;

  // If the subset is of the required size, add it to the result
  if (k == 0) {
    subsets.push_back({});
    return subsets;
  }

  // Generate subsets recursively
  for (size_t i = index; i < set.size(); i++) {
    std::vector<std::vector<int>> smallerSubsets
        = generateSubsets(set, k - 1, i + 1);
    for (auto& smallerSubset : smallerSubsets) {
      smallerSubset.insert(smallerSubset.begin(), set[i]);
      subsets.push_back(smallerSubset);
    }
  }

  return subsets;
}

//==============================================================================
std::vector<std::vector<int>>
GraspSolver::generate_permutations_with_duplicates(
    const std::vector<int>& set, int n) const {
  std::vector<std::vector<int>> permutations;

  if (n == 0) {
    permutations.push_back({});
    return permutations;
  }

  for (size_t i = 0; i < set.size(); i++) {
    std::vector<std::vector<int>> smallerPermutations
        = generate_permutations_with_duplicates(set, n - 1);
    for (auto& smallerPermutation : smallerPermutations) {
      smallerPermutation.insert(smallerPermutation.begin(), set[i]);
      permutations.push_back(smallerPermutation);
    }
  }

  return permutations;
}

//==============================================================================
std::vector<int> GraspSolver::permu(const std::vector<int>& v) const {
  std::vector<int> ret = v;
  std::random_shuffle(ret.begin(), ret.end());
  return ret;
}

//==============================================================================
void GraspSolver::save_yaml(
    const std::string& filename,
    std::priority_queue<
        std::pair<double, std::vector<int>>,
        std::vector<std::pair<double, std::vector<int>>>,
        GraspSolver::Compare> pq) const {

  // go through pq
  YAML::Node result_node;
  int counter = 0;
  while (!pq.empty()) {
    counter++;
    double quality       = pq.top().first;
    std::vector<int> vid = pq.top().second;
    pq.pop();

    YAML::Node node;
    node["quality"] = quality;
    for (size_t i = 0; i < vid.size(); i++) {
      YAML::Node v;
      math::Vector3d pos = m_mesh->get_vertex(vid[i])->get_pos();
      math::Vector3d nor = m_mesh->get_vertex(vid[i])->get_normal();
      v["pos"].push_back(pos[0]);
      v["pos"].push_back(pos[1]);
      v["pos"].push_back(pos[2]);
      v["nor"].push_back(nor[0]);
      v["nor"].push_back(nor[1]);
      v["nor"].push_back(nor[2]);
      node["vertices"].push_back(v);
    }
    result_node["optimal grasps"].push_back(node);
  }

  // output result node
  std::ofstream fout(filename);
  fout << result_node;
  fout.close();
  std::cout << "result written to " << filename << std::endl;
}
