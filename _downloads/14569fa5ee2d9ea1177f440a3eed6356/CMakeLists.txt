# Set local target name
set(target_name ${PROJECT_NAME}-hardware)
set(component_name hardware)

# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")

function(crobot_add_hardware_headers)
  crobot_property_add(CROBOT_HARDWARE_HEADERS ${ARGN})
endfunction(crobot_add_hardware_headers)

function(crobot_add_hardware_sources)
  crobot_property_add(CROBOT_HARDWARE_SOURCES ${ARGN})
endfunction(crobot_add_hardware_sources)

get_property(crobot_hardware_headers GLOBAL PROPERTY CROBOT_HARDWARE_HEADERS)
get_property(crobot_hardware_sources GLOBAL PROPERTY CROBOT_HARDWARE_SOURCES) 

find_package(yaml-cpp REQUIRED)
include_directories(${YAML_INCLUDE_DIRS})

find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# Add target
crobot_add_library(${target_name} ${hdrs} ${srcs} ${crobot_hardware_headers}
                   ${crobot_hardware_sources})
target_link_libraries(
  ${target_name}
  PUBLIC crobot # CRobot base library
         crobot-kdl # CRobot kinematics dynamcis module
         crobot-simulation # CRobot simulation module
         crobot-optimization # CRobot optimization module
         crobot-ik
         ${Boost_SYSTEM_LIBRARY} # Boost ASIO library (WAM, FT sensor)
         ${Boost_THREAD_LIBRARY} # Boost ASIO library (WAM, FT sensor)
         ${Boost_REGEX_LIBRARY} # Boost ASIO library (WAM, FT sensor)
  PRIVATE yaml-cpp # YAML-CPP
          ${OpenCV_LIBS} # OpenCV
          ${LIBXML2_LIBRARIES} # LibXML2 (FT sensor)
)

# Component
add_component(${PROJECT_NAME} ${component_name})
add_component_targets(${PROJECT_NAME} ${component_name} ${target_name})
add_component_dependencies(${PROJECT_NAME} ${component_name} crobot)
# add_component_dependency_packages(${PROJECT_NAME} ${component_name})

# Generate header for this namespace
crobot_get_filename_components(header_names "hardware headers" ${hdrs})
crobot_generate_include_header_file("${CMAKE_CURRENT_BINARY_DIR}/hardware.hpp"
                                    "crobot/hardware/" ${header_names})

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/hardware.hpp
  DESTINATION include/crobot/hardware
  COMPONENT headers)

crobot_format_add(${hdrs} ${srcs} ${crobot_hardware_headers}
                  ${crobot_hardware_sources})
