#ifndef __DETECTION_IO_H__
#define __DETECTION_IO_H__

#include <string>

#include "detection/detection.h"

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

namespace CameraCalibration {

  using AplCam::Detection;

  class DetectionIO {
    public:

      DetectionIO( void );
      virtual ~DetectionIO();

      virtual bool save( const std::string &name, Detection *detection ) = 0;


      static DetectionIO *Create( fs::path &path );

    protected:

  };

  class DetectionIODir : public DetectionIO {
    public:
      DetectionIODir( const fs::path &path );
      virtual ~DetectionIODir();

      virtual bool save( const std::string &name, Detection *detection );


    protected:

      fs::path _path;

  };


}


#endif
