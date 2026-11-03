# ===============================================================================
# CMake settings
# ===============================================================================
cmake_minimum_required(VERSION 3.10.2)

project(crobot CXX)

set(PROJECT_NAME_UPPERCASE CROBOT)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Use CROBOT_SOURCE/BINARY_DIR instead of CMAKE_SOURCE/BINARY_DIR to support the
# case that CROBOT is built as a sub-project in another project.
set(CROBOT_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})
set(CROBOT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR})

include(GNUInstallDirs)

# Variables used in Components.cmake
set(INCLUDE_INSTALL_DIR ${CMAKE_INSTALL_INCLUDEDIR})
set(LIBRARY_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR})
set(CONFIG_INSTALL_DIR "${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME}/cmake")

# Set relative location to install additional documentation
set(CROBOT_ADDITIONAL_DOCUMENTATION_INSTALL_PATH
    "${CMAKE_INSTALL_DATAROOTDIR}/doc/${PROJECT_NAME}")

set(CMAKE_DEBUG_POSTFIX "d")
list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

include(CRobotMacros)

# CMake component helper. It uses the following variables: - LIBRARY_INSTALL_DIR
# - CONFIG_INSTALL_DIR
include(Components)
initialize_component_helpers(${PROJECT_NAME})

# ===============================================================================
# Project settings
# ===============================================================================
set(CROBOT_VERSION_MAJOR 0)
set(CROBOT_VERSION_MINOR 1)
set(CROBOT_VERSION_PATCH 0)
set(CROBOT_VERSION
    ${CROBOT_VERSION_MAJOR}.${CROBOT_VERSION_MINOR}.${CROBOT_VERSION_PATCH})

# ===============================================================================
# Compile Commands Database
# ===============================================================================
# Always generate compile_commands.json for IDE support and static analysis
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# ===============================================================================
# Build options
# ===============================================================================
option(CROBOT_VERBOSE "Whether print detailed information in CMake process" ON)
if(MSVC)
  set(CROBOT_RUNTIME_LIBRARY
      "/MD"
      CACHE STRING "BaseName chosen by the user at CMake configure time")
  set_property(CACHE CROBOT_RUNTIME_LIBRARY PROPERTY STRINGS /MD /MT)
  option(CROBOT_MSVC_DEFAULT_OPTIONS
         "Build CROBOT with default Visual Studio options" OFF)
else()
  option(BUILD_SHARED_LIBS "Build shared libraries" ON)
endif()

option(CROBOT_ASSERT_ENABLED "Enable assertions" ON)
add_definitions(-DCROBOT_ASSERT_ENABLED=1)

# ===============================================================================
# Build options
# ===============================================================================
option(CROBOT_VERBOSE "Whether print detailed information in CMake process" ON)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)
option(CROBOT_BUILD_EXTRAS "Build extra projects" ON)

# ===============================================================================
# Print intro
# ===============================================================================
message(STATUS "")
message(STATUS "============================================")
message(STATUS "                CROBOT ${CROBOT_VERSION}")
message(STATUS "")
message(STATUS "              ISU Robotics Lab")
message(STATUS "============================================")

# ===============================================================================
# Build type settings
# ===============================================================================
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE
      Release
      CACHE
        STRING
        "Choose the type of build, options are: Debug | Release | RelWithDebInfo | MinSizeRel"
        FORCE)
endif()
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPERCASE)

set(BUILD_TYPE_DEBUG FALSE)
set(BUILD_TYPE_RELEASE FALSE)
set(BUILD_TYPE_RELWITHDEBINFO FALSE)
set(BUILD_TYPE_MINSIZEREL FALSE)

if("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "DEBUG")
  set(BUILD_TYPE_DEBUG TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELEASE")
  set(BUILD_TYPE_RELEASE TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELWITHDEBINFO")
  set(BUILD_TYPE_RELWITHDEBINFO TRUE)
elseif("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "MINSIZEREL")
  set(BUILD_TYPE_MINSIZEREL TRUE)
else()
  message(
    WARNING
      "CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} unknown. Valid options are: Debug | Release | RelWithDebInfo | MinSizeRel"
  )
endif()

# Check if ccache is available
find_program(CCACHE_PROGRAM ccache)

if(CCACHE_PROGRAM)
    message(STATUS "ccache found: ${CCACHE_PROGRAM}")
    set(CMAKE_C_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
    set(CMAKE_CXX_COMPILER_LAUNCHER ${CCACHE_PROGRAM})
else()
    message(STATUS "ccache not found, compiling without compiler cache")
endif()

# ===============================================================================
# Find dependencies
# ===============================================================================
include(FindDependencies)

# ===============================================================================
# Check for non-case-sensitive filesystems
# ===============================================================================
execute_process(
  COMMAND ${CMAKE_CURRENT_SOURCE_DIR}/tools/case_sensitive_filesystem
  RESULT_VARIABLE FILESYSTEM_CASE_SENSITIVE_RETURN)
if(${FILESYSTEM_CASE_SENSITIVE_RETURN} EQUAL 0)
  set(FILESYSTEM_CASE_SENSITIVE TRUE)
else()
  set(FILESYSTEM_CASE_SENSITIVE FALSE)
endif()

# ===============================================================================
# Compiler flags
# ===============================================================================
if(CMAKE_COMPILER_IS_GNUCXX)

  if(CROBOT_TREAT_WARNINGS_AS_ERRORS)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror")
  endif()
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -fPIC")
  execute_process(COMMAND ${CMAKE_CXX_COMPILER} -dumpfullversion -dumpversion
                  OUTPUT_VARIABLE GCC_VERSION)
  if(GCC_VERSION VERSION_LESS 7.5.0)
    message(
      FATAL_ERROR
        "The installed g++ version is ${GCC_VERSION}. ${PROJECT_NAME} requires g++ 7.5.0 or greater."
    )
  endif()
  set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
  set(CMAKE_CXX_FLAGS_DEBUG
      "-g -fno-omit-frame-pointer -fno-inline-functions -fno-inline-functions-called-once -fno-optimize-sibling-calls"
  )
  if(CROBOT_FAST_DEBUG)
    set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O1")
  endif()
  set(CMAKE_CXX_FLAGS_RELWITHDEBINFO
      "${CMAKE_CXX_FLAGS_RELEASE} ${CMAKE_CXX_FLAGS_DEBUG}")
  set(CMAKE_CXX_FLAGS_PROFILE "${CMAKE_CXX_FLAGS_DEBUG} -pg")
  set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--no-undefined")

  # Enforce to colorize compilation output
  if(${CROBOT_FORCE_COLORED_OUTPUT})
    add_compile_options(-fdiagnostics-color=always)
  endif()

else()

  message(SEND_ERROR "Compiler[${CMAKE_CXX_COMPILER_ID}] not supported.")

endif()

# ===============================================================================
# Print build summary
# ===============================================================================
if(CROBOT_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Build summary ]")
  message(STATUS "CMAKE_GENERATOR  : ${CMAKE_GENERATOR}")
  message(STATUS "Compiler ID      : ${CMAKE_CXX_COMPILER_ID}")
  message(STATUS "Compiler version : ${CMAKE_CXX_COMPILER_VERSION}")
  message(STATUS "Build type       : ${CMAKE_BUILD_TYPE}")
  message(STATUS "BUILD_SHARED_LIBS: ${BUILD_SHARED_LIBS}")
  message(STATUS "Build gui::osg   : ${CROBOT_BUILD_GUI_OSG}")
  message(STATUS "Install path     : ${CMAKE_INSTALL_PREFIX}")
  message(STATUS "CXX_FLAGS        : ${CMAKE_CXX_FLAGS}")
  if(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "RELEASE")
    message(STATUS "CXX_FLAGS_RELEASE: ${CMAKE_CXX_FLAGS_RELEASE}")
  elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "DEBUG")
    message(STATUS "CXX_FLAGS_DEBUG  : ${CMAKE_CXX_FLAGS_DEBUG}")
  elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "RELWITHDEBINFO")
    message(
      STATUS "CXX_FLAGS_RELWITHDEBINFO: ${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
  elseif(${CMAKE_BUILD_TYPE_UPPERCASE} STREQUAL "PROFILE")
    message(STATUS "CXX_FLAGS_PROFILE: ${CMAKE_CXX_FLAGS_PROFILE}")
  endif()
  message(STATUS "CROBOT_SOURCE_DIR : ${CROBOT_SOURCE_DIR}")
  message(STATUS "CROBOT_BINARY_DIR : ${CROBOT_BINARY_DIR}")
endif(CROBOT_VERBOSE)

# ===============================================================================
# Add sub-directories
# ===============================================================================

add_subdirectory(crobot)

set(CROBOT_IN_SOURCE_BUILD TRUE)

if(TARGET crobot)

  # Add a "tests" target to build unit tests.
  enable_testing()
  add_subdirectory(unittests EXCLUDE_FROM_ALL)

  # Add example subdirectories and an "examples" target.
  add_subdirectory(examples EXCLUDE_FROM_ALL)
  get_property(examples GLOBAL PROPERTY CROBOT_EXAMPLES)
  add_custom_target(examples DEPENDS ${examples})

  if(CROBOT_VERBOSE)
    message(STATUS "")
    message(STATUS "[ Examples ]")
    foreach(example ${examples})
      message(STATUS "Adding example: ${example}")
    endforeach(example ${examples})
  else(CROBOT_VERBOSE)
    list(LENGTH examples examples_length)
    message(STATUS "Adding ${examples_length} examples")
  endif(CROBOT_VERBOSE)
endif()

# Add 'ALL' target that builds everything
set(all_target_candidates tests_and_run)
foreach(target_candidate ${all_target_candidates})
  if(TARGET ${target_candidate})
    list(APPEND all_targets ${target_candidate})
  endif()
endforeach()
foreach(target_candidate ${examples})
  if(TARGET ${target_candidate})
    list(APPEND all_targets ${target_candidate})
  endif()
endforeach()
add_custom_target(ALL DEPENDS ${all_targets})

# ===============================================================================
# Python bindings
# ===============================================================================
option(CROBOT_BUILD_PYTHON_BINDINGS "Build Python bindings" ON)

if(CROBOT_BUILD_PYTHON_BINDINGS)
  message(STATUS "")
  message(STATUS "[ Python bindings ]")
  add_subdirectory(python pycrobot_build)
endif()

# ===============================================================================
# CMake configuration files for components and targets
# ===============================================================================
# Generate and install CMake configuration files for each component <C>: -
# <C>Component.cmake, which defines: - crobot_<C>_DEPENDENCIES: list of
# component dependencies - crobot_<C>_LIBRARIES: list of library targets in this
# component - <C>Targets.cmake, which creates IMPORTED targets
install_component_exports(${PROJECT_NAME})

# ===============================================================================
# Configure files
# ===============================================================================
if(CROBOT_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Configured files ]")
endif()

# Generate and install a Config.cmake file. This file includes the
# <C>Component.cmake and <C>Targets.cmake created above. It also uses the
# following variables:
#
# * PACKAGE_INCLUDE_INSTALL_DIR
# * PACKAGE_INCLUDE_DIRS
get_property(PACKAGE_INCLUDE_DIRS GLOBAL
             PROPERTY "${PROJECT_NAME_UPPERCASE}_INCLUDE_DIRS")

# Generate the CROBOT CMake Config and version files
include(CMakePackageConfigHelpers)
set(CROBOT_CONFIG_IN
    ${CROBOT_SOURCE_DIR}/cmake/${PROJECT_NAME_UPPERCASE}Config.cmake.in)
set(CROBOT_CONFIG_OUT
    ${CROBOT_BINARY_DIR}/${PROJECT_NAME_UPPERCASE}Config.cmake)
set(CROBOT_VERSION_OUT
    ${CROBOT_BINARY_DIR}/cmake/${PROJECT_NAME_UPPERCASE}ConfigVersion.cmake)
if(CROBOT_VERBOSE)
  message(STATUS ${CROBOT_CONFIG_OUT})
  message(STATUS ${CROBOT_VERSION_OUT})
endif()
configure_package_config_file(
  ${CROBOT_CONFIG_IN} ${CROBOT_CONFIG_OUT}
  INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}"
  PATH_VARS INCLUDE_INSTALL_DIR)
write_basic_config_version_file(
  ${CROBOT_VERSION_OUT}
  VERSION ${CROBOT_VERSION}
  COMPATIBILITY SameMajorVersion)
install(FILES ${CROBOT_CONFIG_OUT} ${CROBOT_VERSION_OUT}
        DESTINATION "${CONFIG_INSTALL_DIR}")

# Generate the CROBOT pkg-config
set(PC_CONFIG_IN ${CROBOT_SOURCE_DIR}/cmake/crobot.pc.in)
set(PC_CONFIG_OUT ${CROBOT_BINARY_DIR}/cmake/crobot.pc)
set(PC_CONFIG_INSTALL_DIR
    ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/pkgconfig)
file(RELATIVE_PATH RELATIVE_PATH_TO_INSTALL_PREFIX "${PC_CONFIG_INSTALL_DIR}"
     "${CMAKE_INSTALL_PREFIX}")
if(CROBOT_VERBOSE)
  message(STATUS ${PC_CONFIG_OUT})
endif()
configure_file(${PC_CONFIG_IN} ${PC_CONFIG_OUT} @ONLY)
install(FILES ${PC_CONFIG_OUT} DESTINATION ${PC_CONFIG_INSTALL_DIR})

# Install a Catkin 'package.xml' file. This is required by REP-136.
install(FILES package.xml
        DESTINATION ${CMAKE_INSTALL_DATAROOTDIR}/${PROJECT_NAME})

# ===============================================================================
# Install sample data and examples
# ===============================================================================

# Resources
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/resources"
        DESTINATION ${CROBOT_ADDITIONAL_DOCUMENTATION_INSTALL_PATH})

# Examples source
install(DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/examples"
        DESTINATION ${CROBOT_ADDITIONAL_DOCUMENTATION_INSTALL_PATH})

# ===============================================================================
# Code Formatting
# ===============================================================================
if(CROBOT_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Code Formatting ]")
endif()

find_program(CLANG_FORMAT_EXECUTABLE NAMES clang-format-9 clang-format-11 clang-format-14)

get_property(formatting_files GLOBAL PROPERTY CROBOT_FORMAT_FILES)
list(LENGTH formatting_files formatting_files_length)

if(CLANG_FORMAT_EXECUTABLE)

  if(CROBOT_VERBOSE)
    message(STATUS "Looking for clang-format - found")
  endif()

  message(STATUS "Formatting on ${formatting_files_length} source files.")

  if(formatting_files)
    add_custom_target(
      format
      COMMAND ${CMAKE_COMMAND} -E echo
              "Formatting ${formatting_files_length} files... "
      COMMAND ${CLANG_FORMAT_EXECUTABLE} -style=file -i ${formatting_files}
      COMMAND ${CMAKE_COMMAND} -E echo "Done."
      DEPENDS ${CLANG_FORMAT_EXECUTABLE}
      WORKING_DIRECTORY ${CROBOT_SOURCE_DIR}/crobot)
  else()
    add_custom_target(
      format COMMAND ${CMAKE_COMMAND} -E echo
                     "Warning: Not found any source files to format.")
  endif()

else()
  if(CROBOT_VERBOSE)
    message(
      WARNING
        "Looking for clang-format - NOT found, "
        "please install clang-format-9 to enable automatic code formatting")
  endif()
endif()

# ===============================================================================
# Code Static Analysis
# ===============================================================================
if(CROBOT_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Code Static Analysis ]")
endif()

find_program(
  CLANG_TIDY_EXECUTABLE NAMES clang-tidy-14 clang-tidy-10 clang-tidy-9 clang-tidy-8
                              clang-tidy-7 clang-tidy-6.0 clang-tidy)

if(CLANG_TIDY_EXECUTABLE)
  if(CROBOT_VERBOSE)
    message(STATUS "clang-tidy found: ${CLANG_TIDY_EXECUTABLE}")
  endif()
  set(CMAKE_CXX_CLANG_TIDY "${CLANG_TIDY_EXE}")
else()
  if(CROBOT_VERBOSE)
    message(
      WARNING
        "Looking for clang-tidy - NOT found, "
        "please install clang-tidy-9 to enable automatic code static analysis")
  endif()
endif()

# Add a simple custom target to copy compile_commands.json when requested
add_custom_target(copy_compile_commands
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_BINARY_DIR}/compile_commands.json
        ${CMAKE_SOURCE_DIR}/compile_commands.json
    COMMENT "Copying compile_commands.json to source root"
    VERBATIM
)

# ===============================================================================
# API Document using Doxygen References:
# http://mementocodex.wordpress.com/2013/01/19/how-to-generate-code-documentation-with-doxygen-and-cmake-a-slightly-improved-approach/
# http://www.cmake.org/pipermail/cmake/2007-February/012796.html
# ===============================================================================
if(DOXYGEN_FOUND)

  set(DOXYGEN_DOXYFILE_IN "${PROJECT_SOURCE_DIR}/doxygen/Doxyfile.in")
  set(DOXYGEN_DOXYFILE "${PROJECT_BINARY_DIR}/doxygen/Doxyfile")
  set(DOXYGEN_HTML_INDEX "${PROJECT_BINARY_DIR}/doxygen/html/index.html")
  set(DOXYGEN_OUTPUT_ROOT "${PROJECT_BINARY_DIR}/doxygen/html")
  set(DOXYGEN_GENERATE_TAGFILE "${DOXYGEN_OUTPUT_ROOT}/${PROJECT_NAME}.tag")
  set(DOXYGEN_INCLUDE_PATH "${PROJECT_SOURCE_DIR}")
  set(DOXYGEN_INPUT_ROOT "${PROJECT_SOURCE_DIR}/crobot")
  set(DOXYGEN_EXTRA_INPUTS "${PROJECT_SOURCE_DIR}/doxygen/mainpage.dox")
  set(DOXYGEN_EXCLUDE "${PROJECT_SOURCE_DIR}/crobot/gui")
  set(DOXYGEN_STRIP_FROM_PATH "${CMAKE_CURRENT_SOURCE_DIR}")

  # Generate a Doxyfile. This uses the variables:
  #
  # * DOXYGEN_OUTPUT_ROOT
  # * DOXYGEN_GENERATE_TAGFILE
  # * DOXYGEN_EXTRA_INPUTS
  # * DOXYGEN_INPUT_ROOT
  # * DOXYGEN_EXCLUDE
  # * DOXYGEN_STRIP_FROM_PATH
  configure_file(${DOXYGEN_DOXYFILE_IN} ${DOXYGEN_DOXYFILE} @ONLY)
  file(COPY "${PROJECT_SOURCE_DIR}/doxygen/logo.png"
       DESTINATION ${DOXYGEN_OUTPUT_ROOT})
  set(SED_COMMAND "'s: doxygen=\"[^\"]*\"::g'")
  add_custom_command(
    OUTPUT ${DOXYGEN_HTML_INDEX}
    COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
    COMMAND ${DOXYGEN_EXECUTABLE} -u ${DOXYGEN_DOXYFILE}
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_DOXYFILE}
    # Strip path prefix from all paths in crobot.tag
    COMMAND ${CMAKE_COMMAND} -E echo "Stripping paths from"
            "${DOXYGEN_GENERATE_TAGFILE}"
    COMMAND sed -i s:${DOXYGEN_STRIP_FROM_PATH}::g ${DOXYGEN_GENERATE_TAGFILE}
    # Strip all doxygen="path" HTML tags
    COMMAND ${CMAKE_COMMAND} -E echo "Stripping Doxygen HTML tags"
    COMMAND find "${DOXYGEN_OUTPUT_ROOT}" -type f -name "*.html" -exec sed -i
            ${SED_COMMAND} {} \\$<SEMICOLON>
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/doxygen
    DEPENDS ${DOXYGEN_DOXYFILE})
  # add_custom_target(docs ALL DEPENDS ${DOXYGEN_HTML_INDEX})
  add_custom_target(docs DEPENDS ${DOXYGEN_HTML_INDEX})
  add_custom_target(
    docs_forced
    COMMAND ${CMAKE_COMMAND} -E echo_append "Building API Documentation..."
    COMMAND ${DOXYGEN_EXECUTABLE} -u ${DOXYGEN_DOXYFILE}
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_DOXYFILE}
    COMMAND ${CMAKE_COMMAND} -E echo "Done."
    WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/doxygen)

  # Add the "view_docs" target that opens the generated API documentation.
  if(APPLE)
    set(OPEN_COMMAND "open")
  else()
    set(OPEN_COMMAND "xdg-open")
  endif()

  add_custom_target(
    view_docs
    "${OPEN_COMMAND}" "${DOXYGEN_HTML_INDEX}"
    DEPENDS "${DOXYGEN_HTML_INDEX}"
    COMMENT "Opening documentation in a web browser.")

endif()

# ===============================================================================
# Build Instructions
# ===============================================================================
message(STATUS "")
message(STATUS "Run 'make' to build all the components")
message(STATUS "Run 'make tests' to build all the unittests")
message(STATUS "Run 'make examples' to build all the examples")
message(STATUS "Run 'make view_docs' to see the API documentation")
message(STATUS "Run 'make install' to install all the C++ components")

# ===============================================================================
# END
# ===============================================================================
message(STATUS "")
