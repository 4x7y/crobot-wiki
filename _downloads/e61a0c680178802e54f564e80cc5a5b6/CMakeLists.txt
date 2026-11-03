# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
crobot_add_core_headers(${hdrs} ${detail_hdrs})
crobot_add_core_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
crobot_get_filename_components(header_names "math headers" ${hdrs})
crobot_generate_include_header_file(
  "${CMAKE_CURRENT_BINARY_DIR}/math.hpp"
  "crobot/math/"
  ${header_names}
)

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/math.hpp
  DESTINATION include/crobot/math
  COMPONENT headers
) 
install( 
  FILES ${detail_hdrs} 
  DESTINATION include/crobot/math/detail
  COMPONENT headers
)
 