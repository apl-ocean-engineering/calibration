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

    SonarCalDatum( const Vector2f &_img, float _imgRad, const Vector3f &_sonar, float _sonRad, const string &_name = "" )
        : img( _img ), imgRadius( _imgRad), sonar( _sonar ), sonarRadius( _sonRad ), name( _name )
    {;}

    Vector2f img;
    float imgRadius;
    Vector3f sonar;
    float sonarRadius;

    string name;
  };

  typedef vector<SonarCalDatum,
          Eigen::aligned_allocator< SonarCalDatum > > SonarCalData;

  struct Result {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Result( void )
    {;}
    
    Vector6d pose;

    void scale( float scale )
    {
      pose[3] *= scale;
      pose[4] *= scale;
      pose[5] *= scale;
    }

    bool good;
    float residual;
  };



  SonarCalibrationSolver( void ) {;}

  bool solve( const SonarCalData &, Result & );


 protected:

};




#endif
