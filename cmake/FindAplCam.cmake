

## Try pulling in the aplcam as an external project
include( ExternalProject )
ExternalProject_Add( aplcam
  DOWNLOAD_COMMAND ""
  SOURCE_DIR ${AplCam_SOURCE_DIR}
  PREFIX ${CMAKE_CURRENT_BINARY_DIR}/aplcam
  CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}"
  BUILD_COMMAND ${CMAKE_COMMAND} --build .
  INSTALL_COMMAND ""
)
ExternalProject_Get_Property( aplcam SOURCE_DIR BINARY_DIR )

set( aplcam_INCLUDE_DIR ${SOURCE_DIR}/include )
add_library( libaplcam STATIC IMPORTED )
set_property( TARGET libaplcam PROPERTY IMPORTED_LOCATION ${BINARY_DIR}/lib/libaplcam.a )

find_package_handle_standard_args( aplcam DEFAULT_MSG aplcam_INCLUDE_DIR )
