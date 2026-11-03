# Search all header and source files
file(GLOB hdrs "*.hpp")
file(GLOB srcs "*.cpp")
file(GLOB detail_hdrs "detail/*.hpp")
file(GLOB detail_srcs "detail/*.cpp")
crobot_add_core_headers(${hdrs} ${detail_hdrs})
crobot_add_core_sources(${srcs} ${detail_srcs})

# Generate header for this namespace
crobot_get_filename_components(header_names "common headers" ${hdrs})
crobot_generate_include_header_file("${CMAKE_CURRENT_BINARY_DIR}/common.hpp"
                                    "crobot/common/" ${header_names})

# Install
install(
  FILES ${hdrs} ${CMAKE_CURRENT_BINARY_DIR}/common.hpp
  DESTINATION include/crobot/common
  COMPONENT headers)
install(
  FILES ${detail_hdrs}
  DESTINATION include/crobot/common/detail
  COMPONENT headers)
