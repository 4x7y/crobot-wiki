# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

function(crobot_add_ik_headers)
  crobot_property_add(CROBOT_IK_HEADERS ${ARGN})
endfunction(crobot_add_ik_headers)
  
function(crobot_add_ik_sources) 
  crobot_property_add(CROBOT_IK_SOURCES ${ARGN})
endfunction(crobot_add_ik_sources) 

get_property(crobot_ik_headers GLOBAL PROPERTY CROBOT_IK_HEADERS)
get_property(crobot_ik_sources GLOBAL PROPERTY CROBOT_IK_SOURCES)
 
# Set local target name 
set(target_name ${PROJECT_NAME}-ik) 
set(component_name ik)  

# Add target
crobot_add_library(${target_name} ${hdrs} ${srcs} ${crobot_ik_headers}
                   ${crobot_ik_sources})
target_link_libraries(${target_name} PUBLIC crobot crobot-optimization
                                            crobot-kdl yaml-cpp)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} crobot)

# Generate header for this namespace
crobot_get_filename_components(header_names "ik headers" ${hdrs})
crobot_generate_include_header_file("${CMAKE_CURRENT_BINARY_DIR}/ik.hpp"
                                    "crobot/ik/" ${header_names})

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/ik.hpp
  DESTINATION include/crobot/ik
  COMPONENT headers)

crobot_format_add(${hdrs} ${srcs} ${crobot_ik_headers} ${crobot_ik_sources})
 