#ifndef __CALIBRATOR_H__
#define __CALIBRATOR_H__

#include <opencv2/core/core.hpp>

#include "calibration_opts_common.h"
#include "detection_set.h"
#include "distortion_model.h"
#include "calibration_db.h"
#include "calibration_result.h"
#include "board.h"

namespace AplCam {

  using cv::Size;
  using Distortion::DistortionModel;

  class Calibrator {
    public:

      Calibrator( const CalibrationOptsCommon &opts, const DetectionSet &detSet, const Size &imageSize )
        : _board(NULL), _distModel(NULL), _opts(opts), _detSet( detSet ), _imageSize(imageSize)
      {;}

      ~Calibrator( void )
      {
        if( _board ) delete _board;
        if( _distModel ) delete _distModel;
      }

      void run( void );
      
      void saveDb( const string &dbFile, bool overwriteDb = false );
      void saveDb( CalibrationDb &dbFile, bool overwriteDb = false );

      void saveFile( const string &file ); 
     
      // For this special case, get a non-const DetectionSet
      void updateDetectionPoses( DetectionSet &dets );
 
      CalibrationResult result;


    protected:

      Board *_board;
      DistortionModel *_distModel;

      const CalibrationOptsCommon &_opts;
      const DetectionSet &_detSet;
      const Size &_imageSize;

  };


}

#endif
