

fips_begin_lib( camera_calibration )

  fips_files_ex(. *.cpp)

  fips_libs( ${CERES_LIBRARIES} )

  fips_deps( aplcam )

fips_end_lib()
