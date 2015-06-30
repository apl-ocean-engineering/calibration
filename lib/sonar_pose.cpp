
#include "sonar_pose.h"

using namespace std;
using namespace Eigen;
using namespace cv;


SonarPose::SonarPose( const Vector6d &p )
: _rot( p[0], p[1], p[2] ),
  _trans( p[3], p[4], p[5] )
{;}

SonarPose::SonarPose( const Vec3f &rot, const Vec3f &trans )
: _rot( rot ), _trans( trans )
{;}


Vec3f SonarPose::sonarToImage( const Vec3f &pt )
{
  return rotMat() * pt + _trans;
}

bool SonarPose::write( const string &filename )
{
  FileStorage fs( filename, FileStorage::WRITE  );

  fs << "rot" << _rot;
  fs << "trans" << _trans;
}

SonarPose *SonarPose::Load( const string &filename )
{
  FileStorage fs( filename, FileStorage::READ );

  //Mat rmat, tmat;
  Vec3f rot, trans;

  fs["rot"] >> rot;
  fs["trans"] >> trans;

  return new SonarPose( rot, trans );
}
