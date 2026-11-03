# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

function(crobot_add_gui_headers)
  crobot_property_add(CROBOT_gui_HEADERS ${ARGN})
endfunction(crobot_add_gui_headers)

function(crobot_add_gui_sources)
  crobot_property_add(CROBOT_gui_SOURCES ${ARGN})
endfunction(crobot_add_gui_sources)

get_property(crobot_gui_headers GLOBAL PROPERTY CROBOT_gui_HEADERS)
get_property(crobot_gui_sources GLOBAL PROPERTY CROBOT_gui_SOURCES)

# Set local target name
set(target_name ${PROJECT_NAME}-gui)
set(component_name gui)

# Add target
crobot_add_library(${target_name} ${hdrs} ${srcs} ${crobot_gui_headers}
                   ${crobot_gui_sources})

target_link_libraries(${target_name} PUBLIC crobot glfw ${OPENGL_LIBRARIES} ${GLEW_LIBRARIES})

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} crobot)
# add_component_dependency_packages(${PROJECT_NAME} ${component_name})

# Generate header for this namespace
crobot_get_filename_components(header_names "gui headers" ${hdrs})
crobot_generate_include_header_file("${CMAKE_CURRENT_BINARY_DIR}/gui.hpp"
                                    "crobot/gui/" ${header_names})

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/gui.hpp
  DESTINATION include/crobot/gui
  COMPONENT headers)

crobot_format_add(${hdrs} ${srcs} ${crobot_gui_headers} ${crobot_gui_sources})
