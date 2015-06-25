#ifndef __SONAR_CALIBRATION_SOLVER_H__
#define __SONAR_CALIBRATION_SOLVER_H__

#include <string>
#include <vector>

#include <Eigen/Core>

using Eigen::Vector2f;
using Eigen::Vector3f;

using std::string;
using std::vector;

class SonarCalibrationSolver {
 public:

  struct SonarCalDatum {
    SonarCalDatum( const Vector2f _img, const Vector3f _sonar, const string &_name = "" )
        : img( _img ), sonar( _sonar ), name( _name )
    {;}

    Vector2f img;
    Vector3f sonar;
    string name;
  };

  typedef vector<SonarCalDatum> SonarCalData;



  SonarCalibrationSolver( void ) {;}

  void solve( const SonarCalData & );


 protected:

};


#endif
