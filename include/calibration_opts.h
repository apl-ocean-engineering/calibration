

#ifndef __CALIBRATION_OPTS_H__
#define __CALIBRATION_OPTS_H__

#include <tclap/CmdLine.h>
#include <glog/logging.h>


#include "image.h"
#include "calibration_db.h"
#include "distortion_model.h"
#include "file_utils.h"
using namespace AplCam;
using namespace Distortion;

class CalibrationOpts  {
  public:
    CalibrationOpts();

    bool parseOpts( int argc, char **argv );
    

    string calibrationDb;
    string detectionDb;
    string saveBoardPoses;
    string dataDir;
    string boardName;
    string cameraName;
    string calibrationFile;

    DistortionModel::CalibrationType_t calibType;

    bool overwriteDb;
    bool huberLoss, fixSkew;
    bool doValidate;


    // Functions left over from the old CalibrationOptsCommon
        
      string mkCameraFileName( void ) const
    {
      char strtime[32], buffer[80];
      time_t tt;
      time( &tt );
      struct tm *t2 = localtime( &tt );
      strftime( strtime, 32, "%y%m%d_%H%M%S", t2 );
      snprintf( buffer, 79, "%s_%s.yml", cameraName.c_str(), strtime );
      return  string( buffer );
    }

    const string boardPath( void ) const
    { return dataDir + "/boards/" + boardName + ".yml"; }

    const string cachePath( void ) const
    { return dataDir + "/cache"; }

    const string imageCache( const Image &image ) const
    { return cachePath() + "/" + image.hash() + ".yml"; }

    const string tmpPath( const string &file ) const
    { return dataDir + "/tmp/" + file; }

    const string cameraPath( const string &filename ) const
    {
      string camDir(  dataDir + "/cameras/" + cameraName + "/" );
      if( !directory_exists( camDir ) ) mkdir_p( camDir );
      return camDir + filename;
    }

    virtual int calibFlags( void ) const {
      int flags = 0;

      if( huberLoss ) { flags |= Distortion::CALIB_HUBER_LOSS; }

      // FIX_SKEW disappeared from OpenCV...
      //if( fixSkew )   { flags |= cv::CALIB_FIX_SKEW; }

      return flags;
    }


  protected:

    virtual void doParseCmdLine( TCLAP::CmdLine &cmd, int argc, char **argv );

    bool validateOpts( void );
    virtual bool validate(  void ) { return true; }


 
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


struct IntervalKeyFinder {
  // Ignore end
  IntervalKeyFinder( int start, int interval ) 
    : _start( start ), _interval( interval ) {;}

  int _start, _interval;

  bool operator()( const string &key )
  {
    int s, i;
    int found = sscanf( key.c_str(), "interval(%d,%d", &s, &i );
    if( found != 2 ) return false;

    return (s==_start && i == _interval);
  }
};





#endif

