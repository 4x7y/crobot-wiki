# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

function(crobot_add_collision_headers)
  crobot_property_add(CROBOT_COLLISION_HEADERS ${ARGN})
endfunction(crobot_add_collision_headers)

function(crobot_add_collision_sources)
  crobot_property_add(CROBOT_COLLISION_SOURCES ${ARGN})
endfunction(crobot_add_collision_sources)

get_property(crobot_collision_headers GLOBAL PROPERTY CROBOT_COLLISION_HEADERS)
get_property(crobot_collision_sources GLOBAL PROPERTY CROBOT_COLLISION_SOURCES)

# Set local target name
set(target_name ${PROJECT_NAME}-collision)
set(component_name collision)

# Add target
crobot_add_library(${target_name} ${hdrs} ${srcs} ${crobot_collision_headers}
                   ${crobot_collision_sources})
target_link_libraries(
  ${target_name}
  PUBLIC crobot hpp-fcl::hpp-fcl)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} crobot)
add_component_dependency_packages(${PROJECT_NAME} ${component_name})

# Generate header for this namespace
crobot_get_filename_components(header_names "collision headers" ${hdrs})
crobot_generate_include_header_file("${CMAKE_CURRENT_BINARY_DIR}/collision.hpp"
                                    "crobot/collision/" ${header_names})

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/collision.hpp
  DESTINATION include/crobot/collision
  COMPONENT headers)

crobot_format_add(${hdrs} ${srcs} ${crobot_collision_headers} ${crobot_collision_sources})
