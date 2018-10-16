

fips_begin_lib( camera_calibration )

  fips_src( . )

  fips_libs( ${CERES_LIBRARIES} ${PCL_LIBRARIES} )

  fips_deps( aplcam )

fips_end_lib()
