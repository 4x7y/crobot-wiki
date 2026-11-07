# Reusable mesh data and processing; no dependency on patch trees or grasping.
file(GLOB geometry_headers CONFIGURE_DEPENDS "*.hpp")
set(geometry_sources
  mesh_io.cpp mesh_utils.cpp mesh_preprocessor.cpp
  mesh_refinement.cpp gaussian_curvature.cpp patch_mesh_data.cpp
  mesh_patches.cpp mesh_vertex.cpp icp.cpp surface_icp.cpp)

crobot_add_library(crobot-geometry ${geometry_headers} ${geometry_sources})
target_compile_features(crobot-geometry PUBLIC cxx_std_17)
target_link_libraries(crobot-geometry PUBLIC crobot Eigen3::Eigen Open3D::Open3D)
target_link_libraries(crobot-geometry PRIVATE crobot-optimization)

add_component(${PROJECT_NAME} geometry)
add_component_targets(${PROJECT_NAME} geometry crobot-geometry)
add_component_dependencies(${PROJECT_NAME} geometry crobot optimization)
add_component_dependency_packages(${PROJECT_NAME} geometry Eigen3 Open3D)

install(FILES ${geometry_headers} DESTINATION include/crobot/geometry COMPONENT headers)
crobot_format_add(${geometry_headers} ${geometry_sources})
