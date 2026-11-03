# Patch decomposition and hierarchy construction are a normal CROBOT component.
file(GLOB patch_tree_headers CONFIGURE_DEPENDS "*.hpp")
list(FILTER patch_tree_headers EXCLUDE REGEX "monotone_internal\\.hpp$")
set(patch_tree_sources
  patch_tree.cpp patch_node.cpp mesh_decomposer.cpp patch_tree_io.cpp
  mesh_patch_tree.cpp mesh_patch.cpp monotone_geometry.cpp
  monotone_estimation.cpp monotone_regions.cpp
  monotone_grouping.cpp monotone_decomposition.cpp)

crobot_add_library(crobot-patch_tree ${patch_tree_headers} ${patch_tree_sources})
target_compile_features(crobot-patch_tree PUBLIC cxx_std_17)
target_link_libraries(crobot-patch_tree PUBLIC crobot-geometry yaml-cpp)
find_package(OpenMP QUIET COMPONENTS CXX)
if(OpenMP_CXX_FOUND)
  target_link_libraries(crobot-patch_tree PRIVATE OpenMP::OpenMP_CXX)
endif()

add_component(${PROJECT_NAME} patch_tree)
add_component_targets(${PROJECT_NAME} patch_tree crobot-patch_tree)
add_component_dependencies(${PROJECT_NAME} patch_tree geometry)
add_component_dependency_packages(${PROJECT_NAME} patch_tree YamlCPP)
if(OpenMP_CXX_FOUND)
  add_component_dependency_packages(${PROJECT_NAME} patch_tree OpenMP)
endif()
install(FILES ${patch_tree_headers} DESTINATION include/crobot/patch_tree COMPONENT headers)
crobot_format_add(${patch_tree_headers} ${patch_tree_sources})
