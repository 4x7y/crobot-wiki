# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

# Set local target name
set(target_name ${PROJECT_NAME}-planning)
set(component_name planning)

# Add target
crobot_add_library(${target_name} ${hdrs} ${srcs})
target_link_libraries(${target_name} PUBLIC crobot crobot-kdl crobot-ik ${CGAL_LIBRARIES}
                                            ${CGAL_3RD_PARTY_LIBRARIES})
 
# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} crobot)
add_component_dependency_packages(${PROJECT_NAME} ${component_name})

# Generate header for this namespace
crobot_get_filename_components(header_names "planning headers" ${hdrs})
crobot_generate_include_header_file("${CMAKE_CURRENT_BINARY_DIR}/planning.hpp"
                                    "crobot/planning/" ${header_names})

# Install
install( 
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/planning.hpp
  DESTINATION include/crobot/planning
  COMPONENT headers)

crobot_format_add(${hdrs} ${srcs})
