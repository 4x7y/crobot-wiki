#include "urdf_loader.hpp"

#include <fstream>
#include <iostream>
#include <sstream>
#include <algorithm>
#include <unordered_map>

#include <tinyxml2.h>

#include "crobot/kdl/rigid_body.hpp"
#include "crobot/kdl/rigid_body_joint.hpp"

using namespace tinyxml2;

namespace crobot::kdl {

/**
 * @brief Internal implementation class for UrdfLoader (pimpl idiom)
 */
class UrdfLoader::Impl {
public:
  /**
   * @brief Internal structure to hold URDF link data
   */
  struct UrdfLink {
    std::string name;
    
    // Inertial properties
    double mass = 0.0;
    math::Vector3d center_of_mass = math::Vector3d::Zero();
    math::Matrix3d inertia_matrix = math::Matrix3d::Zero();
    
    // Visual properties
    std::vector<std::string> visual_meshes;
    std::vector<math::Matrix4d> visual_transforms;
    
    // Collision properties
    std::vector<std::string> collision_meshes;
    std::vector<math::Matrix4d> collision_transforms;
  };

  /**
   * @brief Internal structure to hold URDF joint data
   */
  struct UrdfJoint {
    std::string name;
    std::string parent_link;
    std::string child_link;
    JointType type = JointType::FIXED;
    
    // Joint transform (parent to child)
    math::Matrix4d transform = math::Matrix4d::Identity();
    
    // Joint axis (for revolute/prismatic joints)
    math::Vector3d axis = math::Vector3d::UnitZ();
    
    // Joint limits
    bool has_limits = false;
    double lower_limit = 0.0;
    double upper_limit = 0.0;
    double velocity_limit = 0.0;
    double effort_limit = 0.0;
    
    // Joint dynamics
    double damping = 0.0;
    double friction = 0.0;
  };

  // Constructor
  Impl() = default;

  // Main loading functions
  RigidBodyTreePtr load_from_file(const std::string& urdf_file_path);
  RigidBodyTreePtr load_from_string(const std::string& urdf_string, 
                                    const std::string& robot_name);

  // Configuration
  void set_package_path(const std::string& package_name, const std::string& package_path);
  void set_verbose(bool verbose);
  const std::string& get_last_error() const;

private:
  // Parsing functions
  bool parse_urdf_document(XMLDocument* doc, std::string& robot_name);
  UrdfLink parse_link(XMLElement* link_element);
  UrdfJoint parse_joint(XMLElement* joint_element);
  void parse_inertial(XMLElement* inertial_element, UrdfLink& link);
  void parse_visual(XMLElement* visual_element, UrdfLink& link);
  void parse_collision(XMLElement* collision_element, UrdfLink& link);
  math::Matrix4d parse_origin(XMLElement* origin_element);
  JointType convert_joint_type(const std::string& type_str);

  // Tree building functions
  RigidBodyTreePtr build_robot_tree(const std::string& robot_name);
  std::string find_root_link();
  void build_tree_recursive(RigidBodyTreePtr tree, 
                           const std::string& link_name,
                           const std::string& parent_name = "");
  RigidBodyPtr create_rigid_body(const UrdfLink& urdf_link, JointPtr joint = nullptr);
  JointPtr create_joint(const UrdfJoint& urdf_joint);

  // Utility functions
  std::string resolve_package_url(const std::string& package_url);
  bool validate_robot_structure();
  void set_error(const std::string& error);

  // Member variables
  std::unordered_map<std::string, UrdfLink> links_;
  std::unordered_map<std::string, UrdfJoint> joints_;
  std::unordered_map<std::string, std::string> package_paths_;
  
  bool verbose_ = false;
  std::string last_error_;
  
  // Joint name to child link mapping
  std::unordered_map<std::string, std::string> joint_to_child_;
  // Parent link to child joints mapping
  std::unordered_map<std::string, std::vector<std::string>> parent_to_joints_;
};

// UrdfLoader public interface implementation

std::shared_ptr<UrdfLoader> UrdfLoader::create() {
  return std::make_shared<UrdfLoader>();
}

UrdfLoader::UrdfLoader() : pimpl_(std::make_unique<Impl>()) {}

UrdfLoader::~UrdfLoader() = default;

UrdfLoader::UrdfLoader(UrdfLoader&&) noexcept = default;
UrdfLoader& UrdfLoader::operator=(UrdfLoader&&) noexcept = default;

RigidBodyTreePtr UrdfLoader::load_from_file(const std::string& urdf_file_path) {
  return pimpl_->load_from_file(urdf_file_path);
}

RigidBodyTreePtr UrdfLoader::load_from_string(const std::string& urdf_string, 
                                              const std::string& robot_name) {
  return pimpl_->load_from_string(urdf_string, robot_name);
}

void UrdfLoader::set_package_path(const std::string& package_name, 
                                  const std::string& package_path) {
  pimpl_->set_package_path(package_name, package_path);
}

void UrdfLoader::set_verbose(bool verbose) {
  pimpl_->set_verbose(verbose);
}

const std::string& UrdfLoader::get_last_error() const {
  return pimpl_->get_last_error();
}

// UrdfLoader::Impl implementation

RigidBodyTreePtr UrdfLoader::Impl::load_from_file(const std::string& urdf_file_path) {
  // Read file content
  std::ifstream file(urdf_file_path);
  if (!file.is_open()) {
    set_error("Failed to open URDF file: " + urdf_file_path);
    return nullptr;
  }

  std::stringstream buffer;
  buffer << file.rdbuf();
  file.close();

  return load_from_string(buffer.str(), "");
}

RigidBodyTreePtr UrdfLoader::Impl::load_from_string(const std::string& urdf_string, 
                                                    const std::string& robot_name) {
  // Clear previous data
  links_.clear();
  joints_.clear();
  joint_to_child_.clear();
  parent_to_joints_.clear();
  last_error_.clear();

  // Parse XML
  XMLDocument doc;
  XMLError result = doc.Parse(urdf_string.c_str());
  
  if (result != XML_SUCCESS) {
    set_error("XML parsing error: " + std::string(doc.ErrorStr()));
    return nullptr;
  }

  std::string parsed_robot_name = robot_name;
  if (!parse_urdf_document(&doc, parsed_robot_name)) {
    return nullptr;
  }

  if (!validate_robot_structure()) {
    return nullptr;
  }

  return build_robot_tree(parsed_robot_name);
}

void UrdfLoader::Impl::set_package_path(const std::string& package_name, 
                                        const std::string& package_path) {
  package_paths_[package_name] = package_path;
}

void UrdfLoader::Impl::set_verbose(bool verbose) {
  verbose_ = verbose;
}

const std::string& UrdfLoader::Impl::get_last_error() const {
  return last_error_;
}

bool UrdfLoader::Impl::parse_urdf_document(XMLDocument* doc, std::string& robot_name) {
  XMLElement* robot_element = doc->FirstChildElement("robot");
  if (!robot_element) {
    set_error("No robot element found in URDF");
    return false;
  }

  // Get robot name
  const char* name_attr = robot_element->Attribute("name");
  if (name_attr && robot_name.empty()) {
    robot_name = name_attr;
  }
  if (robot_name.empty()) {
    robot_name = "unnamed_robot";
  }

  if (verbose_) {
    std::cout << "Parsing robot: " << robot_name << std::endl;
  }

  // Parse all links
  for (XMLElement* link_elem = robot_element->FirstChildElement("link");
       link_elem; link_elem = link_elem->NextSiblingElement("link")) {
    UrdfLink link = parse_link(link_elem);
    if (!link.name.empty()) {
      links_[link.name] = link;
      if (verbose_) {
        std::cout << "  Parsed link: " << link.name << std::endl;
      }
    }
  }

  // Parse all joints
  for (XMLElement* joint_elem = robot_element->FirstChildElement("joint");
       joint_elem; joint_elem = joint_elem->NextSiblingElement("joint")) {
    UrdfJoint joint = parse_joint(joint_elem);
    if (!joint.name.empty()) {
      joints_[joint.name] = joint;
      joint_to_child_[joint.name] = joint.child_link;
      parent_to_joints_[joint.parent_link].push_back(joint.name);
      if (verbose_) {
        std::cout << "  Parsed joint: " << joint.name 
                  << " (" << joint.parent_link << " -> " << joint.child_link << ")" << std::endl;
      }
    }
  }

  return true;
}

UrdfLoader::Impl::UrdfLink UrdfLoader::Impl::parse_link(XMLElement* link_element) {
  UrdfLink link;
  
  const char* name_attr = link_element->Attribute("name");
  if (!name_attr) {
    return link; // Invalid link
  }
  link.name = name_attr;

  // Parse inertial properties
  XMLElement* inertial_elem = link_element->FirstChildElement("inertial");
  if (inertial_elem) {
    parse_inertial(inertial_elem, link);
  }

  // Parse visual elements
  for (XMLElement* visual_elem = link_element->FirstChildElement("visual");
       visual_elem; visual_elem = visual_elem->NextSiblingElement("visual")) {
    parse_visual(visual_elem, link);
  }

  // Parse collision elements
  for (XMLElement* collision_elem = link_element->FirstChildElement("collision");
       collision_elem; collision_elem = collision_elem->NextSiblingElement("collision")) {
    parse_collision(collision_elem, link);
  }

  return link;
}

UrdfLoader::Impl::UrdfJoint UrdfLoader::Impl::parse_joint(XMLElement* joint_element) {
  UrdfJoint joint;
  
  const char* name_attr = joint_element->Attribute("name");
  const char* type_attr = joint_element->Attribute("type");
  if (!name_attr || !type_attr) {
    return joint; // Invalid joint
  }
  
  joint.name = name_attr;
  joint.type = convert_joint_type(type_attr);

  // Parse parent and child links
  XMLElement* parent_elem = joint_element->FirstChildElement("parent");
  XMLElement* child_elem = joint_element->FirstChildElement("child");
  
  if (parent_elem && child_elem) {
    const char* parent_link = parent_elem->Attribute("link");
    const char* child_link = child_elem->Attribute("link");
    if (parent_link && child_link) {
      joint.parent_link = parent_link;
      joint.child_link = child_link;
    }
  }

  // Parse origin (transform)
  XMLElement* origin_elem = joint_element->FirstChildElement("origin");
  if (origin_elem) {
    joint.transform = parse_origin(origin_elem);
  }

  // Parse axis (for revolute/prismatic joints)
  XMLElement* axis_elem = joint_element->FirstChildElement("axis");
  if (axis_elem) {
    const char* xyz_attr = axis_elem->Attribute("xyz");
    if (xyz_attr) {
      std::istringstream iss(xyz_attr);
      double x, y, z;
      if (iss >> x >> y >> z) {
        joint.axis = math::Vector3d(x, y, z).normalized();
      }
    }
  }

  // Parse limits
  XMLElement* limit_elem = joint_element->FirstChildElement("limit");
  if (limit_elem) {
    joint.has_limits = true;
    limit_elem->QueryDoubleAttribute("lower", &joint.lower_limit);
    limit_elem->QueryDoubleAttribute("upper", &joint.upper_limit);
    limit_elem->QueryDoubleAttribute("velocity", &joint.velocity_limit);
    limit_elem->QueryDoubleAttribute("effort", &joint.effort_limit);
  }

  // Parse dynamics
  XMLElement* dynamics_elem = joint_element->FirstChildElement("dynamics");
  if (dynamics_elem) {
    dynamics_elem->QueryDoubleAttribute("damping", &joint.damping);
    dynamics_elem->QueryDoubleAttribute("friction", &joint.friction);
  }

  return joint;
}

void UrdfLoader::Impl::parse_inertial(XMLElement* inertial_element, UrdfLink& link) {
  // Parse mass
  XMLElement* mass_elem = inertial_element->FirstChildElement("mass");
  if (mass_elem) {
    mass_elem->QueryDoubleAttribute("value", &link.mass);
  }

  // Parse center of mass
  XMLElement* origin_elem = inertial_element->FirstChildElement("origin");
  if (origin_elem) {
    math::Matrix4d com_transform = parse_origin(origin_elem);
    link.center_of_mass = com_transform.block<3, 1>(0, 3);
  }

  // Parse inertia tensor
  XMLElement* inertia_elem = inertial_element->FirstChildElement("inertia");
  if (inertia_elem) {
    double ixx, ixy, ixz, iyy, iyz, izz;
    inertia_elem->QueryDoubleAttribute("ixx", &ixx);
    inertia_elem->QueryDoubleAttribute("ixy", &ixy);
    inertia_elem->QueryDoubleAttribute("ixz", &ixz);
    inertia_elem->QueryDoubleAttribute("iyy", &iyy);
    inertia_elem->QueryDoubleAttribute("iyz", &iyz);
    inertia_elem->QueryDoubleAttribute("izz", &izz);
    
    link.inertia_matrix << ixx, ixy, ixz,
                           ixy, iyy, iyz,
                           ixz, iyz, izz;
  }
}

void UrdfLoader::Impl::parse_visual(XMLElement* visual_element, UrdfLink& link) {
  // Parse geometry
  XMLElement* geometry_elem = visual_element->FirstChildElement("geometry");
  if (geometry_elem) {
    XMLElement* mesh_elem = geometry_elem->FirstChildElement("mesh");
    if (mesh_elem) {
      const char* filename = mesh_elem->Attribute("filename");
      if (filename) {
        link.visual_meshes.push_back(resolve_package_url(filename));
      }
    }
  }

  // Parse origin
  XMLElement* origin_elem = visual_element->FirstChildElement("origin");
  math::Matrix4d transform = math::Matrix4d::Identity();
  if (origin_elem) {
    transform = parse_origin(origin_elem);
  }
  link.visual_transforms.push_back(transform);
}

void UrdfLoader::Impl::parse_collision(XMLElement* collision_element, UrdfLink& link) {
  // Parse geometry
  XMLElement* geometry_elem = collision_element->FirstChildElement("geometry");
  if (geometry_elem) {
    XMLElement* mesh_elem = geometry_elem->FirstChildElement("mesh");
    if (mesh_elem) {
      const char* filename = mesh_elem->Attribute("filename");
      if (filename) {
        link.collision_meshes.push_back(resolve_package_url(filename));
      }
    }
  }

  // Parse origin
  XMLElement* origin_elem = collision_element->FirstChildElement("origin");
  math::Matrix4d transform = math::Matrix4d::Identity();
  if (origin_elem) {
    transform = parse_origin(origin_elem);
  }
  link.collision_transforms.push_back(transform);
}

math::Matrix4d UrdfLoader::Impl::parse_origin(XMLElement* origin_element) {
  math::Matrix4d transform = math::Matrix4d::Identity();
  
  // Parse translation
  const char* xyz_attr = origin_element->Attribute("xyz");
  if (xyz_attr) {
    std::istringstream iss(xyz_attr);
    double x, y, z;
    if (iss >> x >> y >> z) {
      transform.block<3, 1>(0, 3) = math::Vector3d(x, y, z);
    }
  }

  // Parse rotation (RPY)
  const char* rpy_attr = origin_element->Attribute("rpy");
  if (rpy_attr) {
    std::istringstream iss(rpy_attr);
    double roll, pitch, yaw;
    if (iss >> roll >> pitch >> yaw) {
      // Convert RPY to rotation matrix
      // Convert RPY to rotation matrix
      math::AngleAxisd roll_rot(roll, math::Vector3d::UnitX());
      math::AngleAxisd pitch_rot(pitch, math::Vector3d::UnitY());
      math::AngleAxisd yaw_rot(yaw, math::Vector3d::UnitZ());
      math::Matrix3d R = (yaw_rot * pitch_rot * roll_rot).toRotationMatrix();
      transform.block<3, 3>(0, 0) = R;
    }
  }

  return transform;
}

JointType UrdfLoader::Impl::convert_joint_type(const std::string& type_str) {
  if (type_str == "revolute" || type_str == "continuous") {
    return JointType::REVOLUTE;
  } else if (type_str == "prismatic") {
    return JointType::PRISMATIC;
  } else if (type_str == "floating") {
    return JointType::BALL;
  } else {
    return JointType::FIXED;
  }
}

RigidBodyTreePtr UrdfLoader::Impl::build_robot_tree(const std::string& robot_name) {
  auto tree = RigidBodyTree::MakeShared(robot_name);
  
  std::string root_link = find_root_link();
  if (root_link.empty()) {
    set_error("No root link found in robot");
    return nullptr;
  }

  if (verbose_) {
    std::cout << "Building tree from root link: " << root_link << std::endl;
  }

  build_tree_recursive(tree, root_link, "world");
  
  return tree;
}

std::string UrdfLoader::Impl::find_root_link() {
  // Find link that is not a child of any joint
  for (const auto& [link_name, link] : links_) {
    bool is_child = false;
    for (const auto& [joint_name, joint] : joints_) {
      if (joint.child_link == link_name) {
        is_child = true;
        break;
      }
    }
    if (!is_child) {
      return link_name;
    }
  }
  return "";
}

void UrdfLoader::Impl::build_tree_recursive(RigidBodyTreePtr tree, 
                                           const std::string& link_name,
                                           const std::string& parent_name) {
  auto link_it = links_.find(link_name);
  if (link_it == links_.end()) {
    return;
  }

  const UrdfLink& urdf_link = link_it->second;
  
  // Create joint for this link (except root)
  JointPtr joint = nullptr;
  if (!parent_name.empty() && parent_name != "world") {
    // Find joint connecting parent to this link
    for (const auto& [joint_name, urdf_joint] : joints_) {
      if (urdf_joint.child_link == link_name && urdf_joint.parent_link == parent_name) {
        joint = create_joint(urdf_joint);
        break;
      }
    }
  }

  // Create rigid body
  RigidBodyPtr body = create_rigid_body(urdf_link, joint);
  
  // Add to tree
  tree->add_body(body, parent_name);

  // Recursively add children
  auto children_it = parent_to_joints_.find(link_name);
  if (children_it != parent_to_joints_.end()) {
    for (const std::string& child_joint_name : children_it->second) {
      auto joint_it = joints_.find(child_joint_name);
      if (joint_it != joints_.end()) {
        build_tree_recursive(tree, joint_it->second.child_link, link_name);
      }
    }
  }
}

RigidBodyPtr UrdfLoader::Impl::create_rigid_body(const UrdfLink& urdf_link, JointPtr joint) {
  RigidBodyPtr body;
  
  if (joint) {
    body = RigidBody::MakeShared(urdf_link.name, joint);
  } else {
    body = RigidBody::MakeShared(urdf_link.name);
  }

  // Set inertial properties
  body->set_mass(urdf_link.mass);
  body->set_com(urdf_link.center_of_mass);
  body->set_inertia(urdf_link.inertia_matrix);

  return body;
}

JointPtr UrdfLoader::Impl::create_joint(const UrdfJoint& urdf_joint) {
  auto joint = Joint::MakeShared(urdf_joint.name, urdf_joint.type);
  
  // Set joint transform from URDF origin
  joint->set_fixed_transform(urdf_joint.transform);
  
  // Set joint properties based on type
  // Note: Additional joint configuration would go here
  // depending on the Joint class interface
  
  return joint;
}

std::string UrdfLoader::Impl::resolve_package_url(const std::string& package_url) {
  if (package_url.substr(0, 10) == "package://") {
    size_t slash_pos = package_url.find('/', 10);
    if (slash_pos != std::string::npos) {
      std::string package_name = package_url.substr(10, slash_pos - 10);
      std::string relative_path = package_url.substr(slash_pos);
      
      auto it = package_paths_.find(package_name);
      if (it != package_paths_.end()) {
        return it->second + relative_path;
      }
    }
  }
  return package_url;
}

bool UrdfLoader::Impl::validate_robot_structure() {
  // Check that all joint parent/child links exist
  for (const auto& [joint_name, joint] : joints_) {
    if (links_.find(joint.parent_link) == links_.end()) {
      set_error("Joint " + joint_name + " references non-existent parent link: " + joint.parent_link);
      return false;
    }
    if (links_.find(joint.child_link) == links_.end()) {
      set_error("Joint " + joint_name + " references non-existent child link: " + joint.child_link);
      return false;
    }
  }

  // Check for cycles (simplified check)
  if (find_root_link().empty()) {
    set_error("No root link found - robot may have cycles");
    return false;
  }

  return true;
}

void UrdfLoader::Impl::set_error(const std::string& error) {
  last_error_ = error;
  if (verbose_) {
    std::cerr << "URDF Loader Error: " << error << std::endl;
  }
}

// Convenience functions
RigidBodyTreePtr load_urdf_file(const std::string& urdf_file_path, bool verbose) {
  auto loader = UrdfLoader::create();
  loader->set_verbose(verbose);
  return loader->load_from_file(urdf_file_path);
}

RigidBodyTreePtr load_urdf_string(const std::string& urdf_string, 
                                  const std::string& robot_name,
                                  bool verbose) {
  auto loader = UrdfLoader::create();
  loader->set_verbose(verbose);
  return loader->load_from_string(urdf_string, robot_name);
}

} // namespace crobot::kdl
