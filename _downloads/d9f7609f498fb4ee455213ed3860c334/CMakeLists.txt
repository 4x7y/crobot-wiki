# Search all header and source files
file(GLOB hdrs CONFIGURE_DEPENDS "*.hpp")
file(GLOB srcs CONFIGURE_DEPENDS "*.cpp")

function(crobot_add_ekf_headers)
  crobot_property_add(CROBOT_EKF_HEADERS ${ARGN})
endfunction(crobot_add_ekf_headers)

function(crobot_add_ekf_sources)
  crobot_property_add(CROBOT_EKF_SOURCES ${ARGN})
endfunction(crobot_add_ekf_sources)

get_property(crobot_ekf_headers GLOBAL PROPERTY CROBOT_EKF_HEADERS)
get_property(crobot_ekf_sources GLOBAL PROPERTY CROBOT_EKF_SOURCES)

# Set local target name
set(target_name ${PROJECT_NAME}-ekf)
set(component_name ekf)

# Add target
crobot_add_library(${target_name} ${hdrs} ${srcs} ${crobot_ekf_headers}
                   ${crobot_ekf_sources})
target_link_libraries(${target_name} PUBLIC crobot crobot-optimization crobot-kdl)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} crobot optimization kdl)

# Generate header for this namespace
crobot_get_filename_components(header_names "ekf headers" ${hdrs})
crobot_generate_include_header_file("${CMAKE_CURRENT_BINARY_DIR}/ekf.hpp"
                                    "crobot/ekf/" ${header_names})

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/ekf.hpp
  DESTINATION include/crobot/ekf
  COMPONENT headers)

crobot_format_add(${hdrs} ${srcs} ${crobot_ekf_headers} ${crobot_ekf_sources})

