# ======================================
# Mandatory dependencies
# ======================================
if(CROBOT_VERBOSE)
  message(STATUS "")
  message(STATUS "[ Mandatory dependencies for CROBOT core library ]")
endif()

# Eigen
find_package(Eigen3 3.4 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})
message(STATUS "Found Eigen3 in: ${EIGEN3_INCLUDE_DIR}")

# Threads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
crobot_check_required_package(Threads "Threads")

# Spdlog
crobot_find_package(spdlog)
crobot_check_required_package(spdlog "spdlog")

# HPP-FCL
crobot_find_package(HPPFCL REQUIRED)
crobot_check_optional_package(hpp-fcl "crobot-collision" "HPP-FCL")

# QHull
crobot_find_package(QHull REQUIRED)
crobot_check_optional_package(Qhull "crobot-collision" "QHull")

# CCD
crobot_find_package(CCD REQUIRED)
crobot_check_optional_package(ccd "crobot-collision" "CCD")

# IPOPT
crobot_find_package(IPOPT)
crobot_check_optional_package(IPOPT "crobot-optimization" "IPOPT")

# IPOPT
crobot_find_package(OSQP)
crobot_check_optional_package(osqp "crobot-optimization" "OSQP")

# CLP (COIN-OR LP solver)
crobot_find_package(CLP)
crobot_check_optional_package(CLP "crobot-optimization" "CLP (COIN-OR LP)")

# Boost
crobot_find_package(Boost)
crobot_check_optional_package(Boost "crobot-hardware" "Boost" "1.65")

# Kinect Azure Microsoft
# crobot_find_package(AzureKinect)
# crobot_check_optional_package(k4a "crobot-hardware" "AzureKinectSDK")

# LibXML2
crobot_find_package(LibXML2)
crobot_check_optional_package(LibXml2 "crobot-hardware" "LibXML2")

# Mujoco
crobot_find_package(MUJOCO)
crobot_check_optional_package(MUJOCO "crobot-simulation" "MUJOCO")

# GLFW3
find_package(glfw3)
crobot_check_optional_package(glfw3 "crobot-simulation" "glfw3")

# GLEW
find_package(GLEW)
crobot_check_optional_package(GLEW "crobot-simulation" "GLEW")

# YAML-CPP
crobot_find_package(YamlCPP)
crobot_check_optional_package(yaml-cpp "crobot-ik" "YamlCPP")

# TinyXML2
crobot_find_package(TinyXML2)
crobot_check_optional_package(TinyXML2 "crobot-kdl" "TinyXML2")

# MatplotPlusPlus
crobot_find_package(MatplotPlusPlus)
crobot_check_required_package(Matplot++ "MatplotPlusPlus")

# Open3D
crobot_find_package(Open3D)
crobot_check_required_package(Open3D "Open3D")

# --------------------
# Misc. dependencies
# --------------------

# Doxygen
find_package(Doxygen QUIET)
crobot_check_optional_package(DOXYGEN "generating API documentation" "doxygen")
