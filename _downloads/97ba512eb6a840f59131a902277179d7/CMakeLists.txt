# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

function(crobot_add_kdl_headers)
  crobot_property_add(CROBOT_KDL_HEADERS ${ARGN})
endfunction(crobot_add_kdl_headers)

function(crobot_add_kdl_sources)
  crobot_property_add(CROBOT_KDL_SOURCES ${ARGN})
endfunction(crobot_add_kdl_sources)

get_property(crobot_kdl_headers GLOBAL PROPERTY CROBOT_KDL_HEADERS)
get_property(crobot_kdl_sources GLOBAL PROPERTY CROBOT_KDL_SOURCES)

# Set local target name 
set(target_name ${PROJECT_NAME}-kdl)
set(component_name kdl)

# Add target 
crobot_add_library(${target_name} ${hdrs} ${srcs} ${crobot_kdl_headers}
                   ${crobot_kdl_sources})
target_link_libraries(
  ${target_name}
  PUBLIC crobot crobot-collision crobot-optimization yaml-cpp
  PRIVATE tinyxml2::tinyxml2)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} crobot)
add_component_dependency_packages(${PROJECT_NAME} ${component_name})

# Generate header for this namespace
crobot_get_filename_components(header_names "kdl headers" ${hdrs})
crobot_generate_include_header_file("${CMAKE_CURRENT_BINARY_DIR}/kdl.hpp"
                                    "crobot/kdl/" ${header_names})

# Install 
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/kdl.hpp
  DESTINATION include/crobot/kdl
  COMPONENT headers)

crobot_format_add(${hdrs} ${srcs} ${crobot_kdl_headers} ${crobot_kdl_sources})
 