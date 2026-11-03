# Search all header and source files
file(GLOB hdrs CONFIGURE_DEPENDS "*.hpp")
file(GLOB srcs CONFIGURE_DEPENDS "*.cpp")

function(crobot_add_optimization_headers)
  crobot_property_add(CROBOT_OPTIMIZATION_HEADERS ${ARGN})
endfunction(crobot_add_optimization_headers)

function(crobot_add_optimization_sources)
  crobot_property_add(CROBOT_OPTIMIZATION_SOURCES ${ARGN})
endfunction(crobot_add_optimization_sources)

get_property(crobot_optimization_headers GLOBAL
             PROPERTY CROBOT_OPTIMIZATION_HEADERS)
get_property(crobot_optimization_sources GLOBAL
             PROPERTY CROBOT_OPTIMIZATION_SOURCES)

# Set local target name
set(target_name ${PROJECT_NAME}-optimization)
set(component_name optimization)

# Add target
crobot_add_library(
  ${target_name} ${hdrs} ${srcs} ${crobot_optimization_headers}
  ${crobot_optimization_sources})
target_link_libraries(${target_name} PUBLIC crobot PUBLIC ipopt PRIVATE osqp::osqpstatic)

if(HAVE_CLP)
  target_link_libraries(${target_name} PUBLIC PkgConfig::CLP)
  target_compile_definitions(${target_name} PUBLIC CROBOT_HAVE_CLP)
endif()

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} crobot)
# add_component_dependency_packages(${PROJECT_NAME} ${component_name} IPOPT)

# Generate header for this namespace
crobot_get_filename_components(header_names "optimization headers" ${hdrs})
crobot_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/optimization.hpp" "crobot/optimization/"
  ${header_names})

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/optimization.hpp
  DESTINATION include/crobot/optimization
  COMPONENT headers)

crobot_format_add(${hdrs} ${srcs} ${crobot_optimization_headers}
                  ${crobot_optimization_sources})
