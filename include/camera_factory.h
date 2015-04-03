#ifndef __CAMERA_FACTORY_H__
#define __CAMERA_FACTORY_H__

#include <string>

#include "distortion_model.h"

namespace Distortion {

  using std::string;

  class CameraFactory {
    public:
      static DistortionModel *LoadDistortionModel( const string &file );
      static DistortionModel *FromString( const string &file );
      static DistortionModel *Unserialize( FileStorage &fs );


    private:
      CameraFactory() {;}
  };


}

#endif

