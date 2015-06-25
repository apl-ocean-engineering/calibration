#ifndef __SONAR_CALIBRATION_SOLVER_H__
#define __SONAR_CALIBRATION_SOLVER_H__

#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>

#include "sonar_types.h"

using Eigen::Vector2f;
using Eigen::Vector3f;

using Eigen::Vector6d;

using std::string;
using std::vector;

class SonarCalibrationSolver {
 public:

  struct SonarCalDatum {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    SonarCalDatum( const Vector2f &_img, const Vector3f &_sonar, const string &_name = "" )
        : img( _img ), sonar( _sonar ), name( _name )
    {;}

    Vector2f img;
    Vector3f sonar;
    string name;
  };

  typedef vector<SonarCalDatum,
          Eigen::aligned_allocator< SonarCalDatum > > SonarCalData;

  struct Result {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Result( void )
    {;}
    
    Vector6d pose;

    bool good;
    float residual;
  };



  SonarCalibrationSolver( void ) {;}

  bool solve( const SonarCalData &, Result & );


 protected:

};


#endif
