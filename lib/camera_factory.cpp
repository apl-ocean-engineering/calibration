
#include <iostream>

#include "camera_factory.h"

#include "distortion_angular_polynomial.h"

namespace Distortion {

  using namespace std;
  using namespace cv;

  DistortionModel *CameraFactory::LoadDistortionModel( const string &file )
  {
    FileStorage fs( file, FileStorage::READ );
    DistortionModel *out = NULL;

    string type;
    fs["camera_model"] >> type;

    if( type.compare( AngularPolynomial::Name()  ) == 0 ) {
      out = AngularPolynomial::Load( fs );
    } else {
      cerr << "Don't know how to create a camera of type \"" << type << "\"" << endl;
    }

    return out;
  }

  //  public:
  //    Camera( const string &name )
  //      : _name(name), _cam(), _dist()
  //    {; }
  //
  //    const Mat &cameraMatrix( void ) const { return _cam; }
  //    const Mat &cam( void ) const { return _cam; }
  //
  //    const Mat &distCoeffs( void ) const { return _dist; }
  //
  //    bool loadCache( const string &cacheFile )
  //    {
  //      cout << "Loading camera file " << cacheFile << endl;
  //
  //      if( !file_exists( cacheFile ) ) {
  //        cerr << "Cache  file \"" << cacheFile << "\" doesn't exist." << endl;
  //        return false;
  //      }
  //
  //      FileStorage cache( cacheFile, FileStorage::READ );
  //
  //      cache["camera_matrix"] >> _cam;
  //      cache["distortion_coefficients"] >> _dist;
  //
  //      // Insert validation here
  //      //
  //
  //      return true;
  //    }
  //
  //  private:
  //
  //    string _name, _cache;
  //    Mat _cam, _dist;
  //};

}

