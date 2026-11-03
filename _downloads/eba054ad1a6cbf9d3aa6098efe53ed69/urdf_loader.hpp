#pragma once

#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "crobot/kdl/rigid_body_tree.hpp"
#include "crobot/math/matrix.hpp"
#include "crobot/math/quaternion.hpp"

namespace crobot::kdl {

/**
 * @brief URDF parsing and loading utilities for KDL RigidBodyTree
 * 
 * This class provides functionality to parse URDF (Unified Robot Description Format)
 * files and convert them into CRobot KDL RigidBodyTree structures.
 * 
 * Features:
 * - Parse URDF XML files
 * - Convert URDF links to RigidBody objects
 * - Convert URDF joints to Joint objects
 * - Handle inertial properties (mass, center of mass, inertia tensor)
 * - Support visual and collision geometries
 * - Validate robot structure and joint limits
 * 
 * Example usage:
 * @code
 * auto loader = UrdfLoader::create();
 * auto robot_tree = loader->load_from_file("robot.urdf");
 * @endcode
 */
class UrdfLoader {
public:
  /**
   * @brief Create a shared pointer to UrdfLoader
   */
  static std::shared_ptr<UrdfLoader> create();

  /**
   * @brief Default constructor
   */
  UrdfLoader();

  /**
   * @brief Destructor
   */
  ~UrdfLoader();

  // Make non-copyable but movable
  UrdfLoader(const UrdfLoader&) = delete;
  UrdfLoader& operator=(const UrdfLoader&) = delete;
  UrdfLoader(UrdfLoader&&) noexcept;
  UrdfLoader& operator=(UrdfLoader&&) noexcept;

  /**
   * @brief Load robot from URDF file
   * 
   * @param urdf_file_path Path to the URDF file
   * @return RigidBodyTreePtr Loaded robot tree, nullptr on failure
   */
  RigidBodyTreePtr load_from_file(const std::string& urdf_file_path);

  /**
   * @brief Load robot from URDF string
   * 
   * @param urdf_string URDF content as string
   * @param robot_name Optional robot name (extracted from URDF if not provided)
   * @return RigidBodyTreePtr Loaded robot tree, nullptr on failure
   */
  RigidBodyTreePtr load_from_string(const std::string& urdf_string, 
                                    const std::string& robot_name = "");

  /**
   * @brief Set package path for resolving mesh files
   * 
   * @param package_name Package name (e.g., "my_robot_description")
   * @param package_path Absolute path to the package
   */
  void set_package_path(const std::string& package_name, const std::string& package_path);

  /**
   * @brief Enable/disable verbose logging during parsing
   * 
   * @param verbose True to enable verbose output
   */
  void set_verbose(bool verbose);

  /**
   * @brief Get the last error message
   * 
   * @return std::string Error message from last failed operation
   */
  const std::string& get_last_error() const;

private:
  // Forward declaration of implementation class
  class Impl;
  
  // Pointer to implementation (pimpl idiom)
  std::unique_ptr<Impl> pimpl_;
};

/**
 * @brief Convenience function to load URDF file
 * 
 * @param urdf_file_path Path to URDF file
 * @param verbose Enable verbose output
 * @return RigidBodyTreePtr Loaded robot tree, nullptr on failure
 */
RigidBodyTreePtr load_urdf_file(const std::string& urdf_file_path, bool verbose = false);

/**
 * @brief Convenience function to load URDF from string
 * 
 * @param urdf_string URDF content as string
 * @param robot_name Optional robot name
 * @param verbose Enable verbose output
 * @return RigidBodyTreePtr Loaded robot tree, nullptr on failure
 */
RigidBodyTreePtr load_urdf_string(const std::string& urdf_string, 
                                  const std::string& robot_name = "",
                                  bool verbose = false);

} // namespace crobot::kdl
