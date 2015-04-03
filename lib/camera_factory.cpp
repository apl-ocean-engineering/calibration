
#include <iostream>

#include "camera_factory.h"

#include "distortion_angular_polynomial.h"

namespace Distortion {

  using namespace std;
  using namespace cv;

  DistortionModel *CameraFactory::LoadDistortionModel( const string &file )
  {
    FileStorage fs( file, FileStorage::READ );
    return Unserialize( fs );
  }

  DistortionModel *CameraFactory::FromString( const string &str )
  {
    FileStorage fs( str, FileStorage::READ | FileStorage::MEMORY );
    return Unserialize( fs );
  }

  DistortionModel *CameraFactory::Unserialize( FileStorage &fs )
  {
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

}

