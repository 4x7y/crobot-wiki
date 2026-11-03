cmake_minimum_required(VERSION 3.10)

# Find pybind11
find_package(Python3 REQUIRED COMPONENTS Interpreter Development NumPy)
find_package(pybind11 REQUIRED)

# Check pybind11 version compatibility with Python 3.8
if(pybind11_VERSION VERSION_LESS "2.6.0")
  message(FATAL_ERROR "pybind11 version ${pybind11_VERSION} is too old for Python 3.8. Please use pybind11 2.6.0 or newer.")
endif()

# Set Python version compatibility
set(PYBIND11_PYTHON_VERSION ${Python3_VERSION})
set(PYBIND11_CPP_STANDARD -std=c++14)

# Print pybind11 version
message(STATUS "pybind11 version: ${pybind11_VERSION}")

# Print Python version being used (optional)
message(STATUS "Using Python: ${Python3_EXECUTABLE}")
message(STATUS "Includes: ${Python3_INCLUDE_DIRS}")
message(STATUS "Library: ${Python3_LIBRARIES}")

execute_process(
    COMMAND ${Python3_EXECUTABLE} -c "import sys; print(sys.version)"
    OUTPUT_VARIABLE PYTHON_VERSION_OUTPUT
    OUTPUT_STRIP_TRAILING_WHITESPACE
)
message(STATUS "Python sys.version: ${PYTHON_VERSION_OUTPUT}")

# Add the Python bindings
pybind11_add_module(pycrobot 
    src/pycrobot.cpp
    src/control_bindings.cpp
    src/kdl_bindings.cpp
    src/math_bindings.cpp
    src/optimization_bindings.cpp
    src/ik_bindings.cpp
)

# Link against the CRobot libraries
target_link_libraries(pycrobot PRIVATE
    crobot
    crobot-kdl
    crobot-control
    crobot-optimization
    crobot-ik
    Python3::Python
    pybind11::module
)
target_include_directories(pycrobot PRIVATE ${Python3_INCLUDE_DIRS})

# Test the Python bindings
enable_testing()
add_test(
    NAME pybind_tests
    COMMAND ${Python3_EXECUTABLE} -m pytest ${CMAKE_SOURCE_DIR}/python/tests
)
set_tests_properties(pybind_tests PROPERTIES
    ENVIRONMENT "PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}"
)

# Install the Python module
install(TARGETS pycrobot LIBRARY DESTINATION ${Python3_SITEARCH})

# Install stub files and py.typed for mypy support
install(FILES
    stub/py.typed
    DESTINATION ${Python3_SITEARCH}/pycrobot
)

# Install type stubs for all submodules
install(DIRECTORY stub/
    DESTINATION ${Python3_SITEARCH}/pycrobot
    FILES_MATCHING PATTERN "*.pyi"
    PATTERN "__pycache__" EXCLUDE
    PATTERN "*.py" EXCLUDE  # Exclude .py files to avoid conflicts
)
