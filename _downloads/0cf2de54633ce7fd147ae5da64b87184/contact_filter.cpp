#include "crobot/grasping/contact_filter.hpp"

#include "crobot/common/assert.hpp"

using namespace crobot;

bool crobot::grasping::are_patch_centers_nearby(
    const geometry::MeshPatches& mesh, const std::vector<int>& patch_labels) {
  CROBOT_ASSERT_ERROR(
      patch_labels.size() == 3, "Only support 3 patches for now");

  if (!mesh.is_center_defined(patch_labels)) {
    return false;
  }
  std::vector<math::Vector3d> centers;
  for (int label : patch_labels) {
    centers.push_back(mesh.get_patch_center(label)->get_pos());
  }

  const double thumb_dist = 120 * mesh.get_scale(); // mm
  const double other_dist = 40 * mesh.get_scale();  // mm
  const double min_dist   = 10;
  std::vector<int> dist_range_counter(4, 0);

  for (size_t i = 0; i < centers.size(); i++) {
    for (size_t j = i + 1; j < centers.size(); j++) {
      double dist = (centers[i] - centers[j]).norm();
      if (dist > thumb_dist) {
        dist_range_counter[3]++;
      }
      else if (dist > other_dist) {
        dist_range_counter[2]++;
      }
      else if (dist > min_dist) {
        dist_range_counter[1]++;
      }
      else {
        dist_range_counter[0]++;
      }
    }
  }

  if (dist_range_counter[3] + dist_range_counter[0]
      > 0) { // any pair of patches are too far
    return false;
  }
  if (dist_range_counter[1] > 0) { // at least one pair needs to be close
    return true;
  }
  return false;
}
