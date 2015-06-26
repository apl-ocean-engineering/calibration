
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


bool SonarPose::write( const string &filename )
{
  FileStorage fs( filename, FileStorage::WRITE  );

  fs << "rot" << _rot;
  fs << "trans" << _trans;
}

SonarPose *SonarPose::Load( const string &filename )
{
  FileStorage fs( filename, FileStorage::READ );

  Mat rmat, tmat;
  fs["rot"] >> rmat;
  fs["trans"] >> tmat;


  Vec3f rot, 
        trans;

  rmat.convertTo( rot, CV_32F );
  tmat.convertTo( trans, CV_32F );

  return new SonarPose( rot, trans );
}


