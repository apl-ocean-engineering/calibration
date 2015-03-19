#ifndef __CAMERA_FACTORY_H__
#define __CAMERA_FACTORY_H__

#include <string>

#include "distortion_model.h"

namespace Distortion {

  using std::string;

  class CameraFactory {
    public:
      static DistortionModel *LoadDistortionModel( const string &file );

    private:
      CameraFactory() {;}
  };


}

#endif

