# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

function(crobot_add_simulation_headers)
  crobot_property_add(CROBOT_SIMULATION_HEADERS ${ARGN})
endfunction(crobot_add_simulation_headers)

function(crobot_add_simulation_sources)
  crobot_property_add(CROBOT_SIMULATION_SOURCES ${ARGN})
endfunction(crobot_add_simulation_sources)

get_property(crobot_simulation_headers GLOBAL
             PROPERTY CROBOT_SIMULATION_HEADERS)
get_property(crobot_simulation_sources GLOBAL
             PROPERTY CROBOT_SIMULATION_SOURCES)

# Set local target name
set(target_name ${PROJECT_NAME}-simulation)
set(component_name simulation)

# Add target
crobot_add_library(${target_name} ${hdrs} ${srcs} ${crobot_simulation_headers}
                   ${crobot_simulation_sources})
target_link_libraries(
  ${target_name}
  PUBLIC crobot crobot-kdl ${MUJOCO_LIBRARIES}
  OpenGL::OpenGL OpenGL::GLX glfw ${GLEW_LIBRARIES})

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} crobot)
add_component_dependency_packages(${PROJECT_NAME} crobot MUJOCO)

# Generate header for this namespace
crobot_get_filename_components(header_names "simulation headers" ${hdrs})
crobot_generate_include_header_file("${CMAKE_CURRENT_BINARY_DIR}/simulation.hpp"
                                    "crobot/simulation/" ${header_names})

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/simulation.hpp
  DESTINATION include/crobot/simulation
  COMPONENT headers)

crobot_format_add(${hdrs} ${srcs} ${crobot_simulation_headers}
                  ${crobot_simulation_sources})
