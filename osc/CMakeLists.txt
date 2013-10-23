message( "=== gvfosc ===" )

include(FindPkgConfig)
pkg_check_modules(liblo REQUIRED liblo)

include_directories(
  ${CMAKE_BINARY_DIR}
  ${liblo_INCLUDE_DIRS}
  ${gvf_SOURCE_DIR}/src/
  ${gvf_SOURCE_DIR}/Eigen/
)

link_directories(
   ${liblo_LIBRARY_DIRS}
)


##########################################################################
# gvfosc
##########################################################################

set(gvfosc_SRCS
  gvfosc.cpp
)

add_executable( gvfosc ${gvfosc_SRCS} )

target_link_libraries(gvfosc ${liblo_LIBRARIES} gvf )

install(TARGETS gvfosc DESTINATION bin)