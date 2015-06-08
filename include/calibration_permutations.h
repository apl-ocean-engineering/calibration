
#ifndef __CALIBRATION_PERMUTATIONS_H__
#define __CALIBRATION_PERMUTATIONS_H__

#include "calibration_db.h"
#include "calibration_opts_common.h"
#include "calibrator.h"
using namespace AplCam;

#include <tclap/CmdLine.h>
#include <glog/logging.h>

class CalibrationOpts : public AplCam::CalibrationOptsCommon {

  public:

    CalibrationOpts();

    string calibrationDb;
    string detectionDb;
    string saveBoardPoses;

    bool fixSkew, overwriteDb;

    bool parseOpts( int argc, char **argv );
    
    virtual void doParseCmdLine( TCLAP::CmdLine &cmd, int argc, char **argv );
    
    virtual bool validate(  void );
 
};




struct RandomKeyCounter {
  RandomKeyCounter( int i ) : _count(i) {;}
  int _count;

  bool operator()( const string &key )
  {
    int c;
    int found = sscanf( key.c_str(), "random(%d)", &c );
    if( found != 1 ) return false;

    return c == _count;
  }
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
