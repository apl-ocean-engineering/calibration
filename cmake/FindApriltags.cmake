
include( ExternalProject )

if( Apriltags_SOURCE_DIR )
  message( "Using Apriltags from directory at ${Apriltags_SOURCE_DIR}" )
  ExternalProject_Add( apriltags
    DOWNLOAD_COMMAND ""
    SOURCE_DIR ${Apriltags_SOURCE_DIR}
    PREFIX ${PROJECT_BINARY_DIR}/apriltags
    CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}"
    BUILD_COMMAND ${CMAKE_COMMAND} --build . -- apriltags
    INSTALL_COMMAND ""
  )
else()
  message( "Using Apriltags from git repo" )
  message( "To use local version set CMake variable Apriltags_SOURCE_DIR")
  ExternalProject_Add( apriltags
    GIT_REPOSITORY "https://github.com/amarburg/apriltags.git"
    GIT_TAG subtag_detection
    PREFIX ${PROJECT_BINARY_DIR}/apriltags
    CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}"
    BUILD_COMMAND ${CMAKE_COMMAND} --build .
    INSTALL_COMMAND ""
  )
endif()

ExternalProject_Add_Step( apriltags forceconfigure
     COMMAND ${CMAKE_COMMAND} -E echo "Force build of apriltags"
     DEPENDEES configure
     DEPENDERS build
     ALWAYS 1)

ExternalProject_Get_Property( apriltags SOURCE_DIR BINARY_DIR )

set( apriltags_INCLUDE_DIRS ${SOURCE_DIR}/include )

message( "Apriltags source is at ${SOURCE_DIR}" )
add_library( libapriltags STATIC IMPORTED )
set_property( TARGET libapriltags PROPERTY IMPORTED_LOCATION ${BINARY_DIR}/libapriltags.a )
set( apriltags_LIBS libapriltags )
set( Apriltags_SOURCE_DIR ${SOURCE_DIR} )

find_package_handle_standard_args( apriltags DEFAULT_MSG apriltags_INCLUDE_DIRS apriltags_LIBS  )
