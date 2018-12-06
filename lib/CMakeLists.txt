

fips_begin_lib( camera_calibration )

  fips_src( . )
  
  fips_deps( aplcam )

  fips_libs( ${CERES_LIBRARIES} ${PCL_LIBRARIES} )

  if(OpenMP_CXX_FOUND)
    fips_libs(OpenMP::OpenMP_CXX)
  endif()

fips_end_lib()
