
# Enable multi-threaded compilation. We do this here and not in the root folder
# since the examples and the tutorials do not have enough source files to
# benefit from this.
if(MSVC AND ${CMAKE_CXX_COMPILER_ID} STREQUAL "MSVC")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP")
endif()

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CROBOT_BINARY_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CROBOT_BINARY_DIR}/lib")

# ===============================================================================
# Source directories
# ===============================================================================
# common
 
# Component. This should be called before adding subdirectories.
add_component(${PROJECT_NAME} crobot)

function(crobot_add_core_headers)
  crobot_property_add(CROBOT_CORE_HEADERS ${ARGN})
endfunction(crobot_add_core_headers)

function(crobot_add_core_sources)
  crobot_property_add(CROBOT_CORE_SOURCES ${ARGN})
endfunction(crobot_add_core_sources)

# Add subdirectories
add_subdirectory(common)
add_subdirectory(math)
add_subdirectory(optimization)
add_subdirectory(ik)
add_subdirectory(control)
add_subdirectory(hardware)
add_subdirectory(simulation)
add_subdirectory(geometry)
add_subdirectory(patch_tree)
add_subdirectory(grasping)
add_subdirectory(planning)
add_subdirectory(gui)
add_subdirectory(kdl)
add_subdirectory(collision)
add_subdirectory(ekf)


set(CROBOT_CONFIG_HPP_IN ${CROBOT_SOURCE_DIR}/crobot/config.hpp.in)
set(CROBOT_CONFIG_HPP_OUT ${CROBOT_BINARY_DIR}/crobot/config.hpp)
if(CROBOT_VERBOSE)
  message(STATUS ${CROBOT_CONFIG_HPP_OUT})
endif()
configure_file(${CROBOT_CONFIG_HPP_IN} ${CROBOT_CONFIG_HPP_OUT} @ONLY)
install(FILES ${CROBOT_CONFIG_HPP_OUT} DESTINATION include/crobot)

# Print building component
get_property(components GLOBAL PROPERTY ${PROJECT_NAME}_COMPONENTS)
if(CROBOT_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Components ]")
  foreach(component ${components})
    message(STATUS "Adding component: ${component}")
  endforeach()
else()
  list(LENGTH components components_length)
  message(STATUS "Adding ${components_length} components including "
                 "the default component 'crobot'")
endif()

# Set header and source files
get_filename_component(crobot_hpp "crobot.hpp" ABSOLUTE)
crobot_add_core_headers(${crobot_hpp})
get_property(crobot_core_headers GLOBAL PROPERTY CROBOT_CORE_HEADERS)
get_property(crobot_core_sources GLOBAL PROPERTY CROBOT_CORE_SOURCES)

# Add target
crobot_add_library(crobot ${crobot_core_headers} ${crobot_core_sources})
target_include_directories(
  crobot BEFORE
  PUBLIC $<BUILD_INTERFACE:${CROBOT_SOURCE_DIR}>
         $<BUILD_INTERFACE:${CROBOT_BINARY_DIR}> $<INSTALL_INTERFACE:include>)

target_compile_features(crobot PUBLIC cxx_std_17)
if(CMAKE_COMPILER_IS_GNUCXX AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS 8.0)
  target_link_libraries(crobot PUBLIC "stdc++fs")
endif()

target_link_libraries(
  crobot
  PUBLIC ${CMAKE_DL_LIBS} # Name of library containing dlopen and dlclose.
         Eigen3::Eigen # Eigen3, a library for linear algebra.
         spdlog::spdlog # Logging library
         yaml-cpp
         Matplot++::matplot)

# Default component
add_component_targets(${PROJECT_NAME} crobot crobot)
add_component_dependency_packages(${PROJECT_NAME} crobot Eigen3 spdlog)

if(MSVC)
  set_target_properties(${target} PROPERTIES STATIC_LIBRARY_FLAGS_RELEASE
                                             "/LTCG")
endif() 

install(
  FILES crobot.hpp 
  DESTINATION include/crobot/
  COMPONENT headers)

crobot_format_add(${crobot_core_headers} ${crobot_core_sources})
