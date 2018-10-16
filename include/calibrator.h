#ifndef __CALIBRATOR_H__
#define __CALIBRATOR_H__

#include <opencv2/core/core.hpp>

#include "AplCam/detection_set.h"
#include "AplCam/calibration_db.h"
#include "AplCam/calibration_result.h"
#include "AplCam/board/board.h"
using namespace AplCam;

#include "AplCam/distortion/distortion_model.h"
using Distortion::DistortionModel;


#include "calibration_opts.h"

using cv::Size;



class Calibrator {
  public:

    Calibrator( const CalibrationOpts &opts, const DetectionSet &detSet, const Size &imageSize )
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

    const CalibrationOpts &_opts;
    const DetectionSet &_detSet;
    const Size &_imageSize;

};

struct CalibrateFunctor {
  public:
    CalibrateFunctor( const CalibrationOpts &opts, const Size &imageSize, CalibrationDb &db, vector< DetectionSet * > &detSets )
      : _opts( opts ), _imageSize( imageSize ), _db( db ), _detSets( detSets )
    {;}

    const CalibrationOpts &_opts;
    const Size &_imageSize;
    CalibrationDb &_db;
    vector< DetectionSet *> _detSets;

    void operator()(void ) const {
      size_t end = _detSets.size();
      for( size_t i = 0; i != end; ++i ) {

        DetectionSet &detSet( *_detSets[i] );

        Calibrator cal( _opts, *_detSets[i], _imageSize );
        cal.run();

        //  Want to measure failure rate, Save it regardless of whether it's good.
        cal.saveDb( _db );

        // If it's the "all" set, consider saving the board posees
        if( _opts.saveBoardPoses.length() > 0 && detSet.name().compare("all") == 0 ) {
          DetectionDb savedPoses( _opts.saveBoardPoses, true );
          cal.updateDetectionPoses( detSet );
          savedPoses.save( detSet );
        }
      }
    }
};




#endif
