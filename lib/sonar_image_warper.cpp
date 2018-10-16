
#include "sonar_image_warper.h"

#include "AplCam/distortion/camera_factory.h"


namespace camera_calibration {

  using namespace cv;
  using namespace std;
  using namespace Distortion;
  using namespace AplCam;

  SonarImageWarper::SonarImageWarper( DistortionModel *cam, AplCam::SonarPose *pose )
    : _cam( cam ), _pose( pose )
  {;}

  SonarImageWarper::SonarImageWarper( const string &cameraCalFile, const string &sonarPoseFile )
    :_cam( NULL ), _pose( NULL )
  {
    _cam = CameraFactory::LoadDistortionModel( cameraCalFile );
    _pose = SonarPose::Load( sonarPoseFile );

  assert( _cam != NULL );
  assert( _pose != NULL );

  //  LOG(INFO) << _pose->rot();
  //  LOG(INFO) << _pose->trans();
  }

  Vec2f SonarImageWarper::sonarToImage(  float x, float y, float z )
  {
    return sonarToImage( Vec3f( x,y,z ));
  }

  Vec2f SonarImageWarper::sonarToImage( const Vec3f &s )
  {
    // Transform point to image frame
    Vec3f inCamFrame( _pose->sonarToImage( s ) );
    Vec2d inCamDist( _cam->distort( inCamFrame ) );
    //Vec2d inImgDist( _cam->image( Vec2f(inImgFrame[0]/inImgFrame[2], inImgFrame[1]/inImgFrame[2] ) ) );
    Vec2f inImg( _cam->image(inCamDist) );

  //LOG(INFO) << "sonar: " << s << " in cam frame: " << inCamFrame << " distorted: " << inCamDist << " imaged: " << inImg;

    return inImg;
  }

}
