
#include "sonar_pose.h"

using namespace std;
using namespace Eigen;
using namespace cv;


SonarPose::SonarPose( const Vector6d &p, float scale )
: _rot( p[0], p[1], p[2] ),
  _trans( p[3], p[4], p[5] ),
  _scale( scale )
{;}

SonarPose::SonarPose( const Vec3f &rot, const Vec3f &trans, float scale )
: _rot( rot ), _trans( trans ), _scale( scale )
{;}


Vec3f SonarPose::sonarToImage( const Vec3f &pt )
{
  return rotMat() * (pt * _scale) + _trans;
}

bool SonarPose::write( const string &filename )
{
  FileStorage fs( filename, FileStorage::WRITE  );

  fs << "rot" << _rot;
  fs << "trans" << _trans;
  fs << "scale" << _scale;
}

SonarPose *SonarPose::Load( const string &filename )
{
  FileStorage fs( filename, FileStorage::READ );

  //Mat rmat, tmat;
  Vec3f rot, trans;
float scale;

  fs["rot"] >> rot;
  fs["trans"] >> trans;
  fs["scale"] >> scale;

  return new SonarPose( rot, trans, scale );
}
