# Search all header and source files in current directory
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

# Combine all sources
set(all_hdrs ${hdrs})
set(all_srcs ${srcs})

# Set local target name
set(target_name ${PROJECT_NAME}-grasping)
set(component_name grasping)

# Grasp generation consumes geometry, hierarchy, and optimization components.
crobot_add_library(${target_name} ${all_hdrs} ${all_srcs})
target_link_libraries(${target_name} PUBLIC crobot-geometry crobot-patch_tree crobot-optimization)

# Public headers use the repository/include root inherited from crobot.

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} geometry patch_tree optimization)
add_component_dependency_packages(${PROJECT_NAME} ${component_name})

# Generate header for this namespace
crobot_get_filename_components(header_names "grasping headers" ${hdrs})
crobot_generate_include_header_file("${CMAKE_CURRENT_BINARY_DIR}/grasping.hpp"
                                    "crobot/grasping/" ${header_names})

# Install main headers
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/grasping.hpp
  DESTINATION include/crobot/grasping
  COMPONENT headers)

crobot_format_add(${all_hdrs} ${all_srcs})
