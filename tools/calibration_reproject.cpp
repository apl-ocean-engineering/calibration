#include <cctype>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <getopt.h>

#include <iostream>

#ifdef USE_TBB
#include "tbb/tbb.h"
using namespace tbb;
#endif

#include "file_utils.h"
#include "board.h"
#include "detection.h"
#include "detection_set.h"
#include "image.h"

#include "distortion_model.h"
#include "camera_factory.h"
using namespace Distortion;

#include "calibration_db.h"
#include "calibration_opts_common.h"
#include "calibrator.h"
using namespace AplCam;

#include "video_splitters/video_splitter_opts.h"
#include "video_splitters/video_splitters.h"
using namespace AplCam::VideoSplitters;


using namespace cv;
using namespace std;

using kyotocabinet::HashDB;
using kyotocabinet::DB;

class CalibrationReprojectOpts : public AplCam::CalibrationOptsCommon {

  public:

    CalibrationReprojectOpts()
      : CalibrationOptsCommon(), 
      calibrationDb(), resultsDb(), 
      referenceDb()
  {;}

    string calibrationDb;
    string resultsDb;
    string referenceDb;

    //== Option parsing and help ==
    string help( void )
    {
      stringstream strm;

      return strm.str();
    }


    bool parseOpts( int argc, char **argv, string &msg )
    {
      stringstream msgstrm;

      static struct option long_options[] = {
        { "data-directory", true, NULL, 'd' },
        { "calibration-db", required_argument, NULL, 'Z' },
        { "reference-db", required_argument, NULL, 'r' },
        { "results-db", required_argument, NULL, 'R' },
        { "help", false, NULL, '?' },
        { 0, 0, 0, 0 }
      };


      int indexPtr;
      int optVal;
      string c;

      // The '+' option ensures it stops on the first non-conforming option. Required for the
      //   cmd opt1 opt2 opt3 verb verb_opt1 files ...
      // pattern I'm using
      while( (optVal = getopt_long( argc, argv, "d:r:R:Z:?", long_options, &indexPtr )) != -1 ) {
        switch( optVal ) {
          case 'Z':
            calibrationDb = optarg;
            break;
          case 'd':
            dataDir = optarg;
            break;
          case 'R':
            resultsDb = optarg;
            break;
          case 'r':
            referenceDb = optarg;
            break;
          case '?': 
            cout << help() << endl;;
            return false;
            break;
          default:
            return false;

        }
      }

      if( !validate( msg ) ) return false;

      return true;
    }


    bool checkForDb( const string &db, const string &name, string &msg )
    {
      if( db.empty() ) {
        msg = string("Must specify ") + name + " db";
        return false;
      }

      if( !file_exists( db ) ) {
        msg = string("Can't find ") + name + " db " + db;
        return false;
      }

      return true;
    }

    virtual bool validate( string &msg )
    {

      if( !checkForDb( calibrationDb, "calibration", msg ) ) return false;
      if( !checkForDb( referenceDb, "reference", msg ) ) return false;

      if( resultsDb.empty() ) {
        msg = "Must specify results db";
        return false;
      }

      return true;
    }

};



class CalibrationReprojectFunctor {
  public:
  CalibrationReprojectFunctor( const ObjectPointsVecVec &obj, const ImagePointsVecVec &img,
      const RotVec &rv, const TransVec &tv,
      CalibrationDb &db,
      kyotocabinet::HashDB &results )
    : _objectPoints( obj ),
      _imagePoints( img ),
      _rvecs( rv ),
      _tvecs( tv ),
      _keys(),
      _calDb( db ),
      _results( results )
  {;}

void setKeys( const vector< string > &k )
{ _keys = k; }

  const ObjectPointsVecVec &_objectPoints;
  const ImagePointsVecVec &_imagePoints;
  const RotVec &_rvecs;
  const TransVec &_tvecs;
  vector< string > _keys;
  CalibrationDb &_calDb;
  kyotocabinet::HashDB &_results;


#ifdef USE_TBB
  void operator()( const tbb::blocked_range<size_t> &r ) const
  {
    size_t end = r.end();
    for( size_t i = r.begin(); i != end; ++i ) {
#else
  void operator()( void )
  {
    size_t end = _keys.size();
    for( size_t i = 0; i < end; ++i ) {
#endif
      
      const string &key( _keys[i] );
      string value;
      _calDb.get( key, &value );

      Result result( _objectPoints.size() );

      DistortionModel *distModel = CameraFactory::FromString( value );

      if( !distModel ) {
        cout << "Couldn't create camera from key " << key << endl;
        continue;
      }

      result.rms = distModel->reprojectionError( _objectPoints, 
          _rvecs, _tvecs, _imagePoints, 
          result.reprojErrors );

      // Hm, cant get this from anyhere
      result.numPoints = 0;
      for( size_t i = 0; i < _objectPoints.size(); ++i ) 
        result.numPoints += _objectPoints[i].size();

      result.numImages = _objectPoints.size();

      cout << "Saving " << key << endl;
      if( !_results.set( key, result.toString() ) ) {
        cerr << "Error saving results for key " << key << endl;
      }

    }
  }
};


int main( int argc, char** argv )
{

  CalibrationReprojectOpts opts;

  string optsMsg;
  if( !opts.parseOpts( argc, argv, optsMsg ) ) {
    cout << optsMsg << endl;
    exit(-1);
  }

  DetectionDb refDets;
  if( ! refDets.open( opts.referenceDb ) ) {
    cerr << "Error opening reference detection db: " << refDets.error().name() << endl;
    exit(-1);
  }

  DetectionSet refDetections;
  AllGoodVideoSplitter vs;
  vs.generate( refDets, refDetections );

  cout << "Loaded reference detection set with " << refDetections.size() << " frames." << endl;

  ObjectPointsVecVec objectPoints( refDetections.objectPoints() );
  ImagePointsVecVec imagePoints( refDetections.imagePoints() );
  RotVec            rvecs( refDetections.rvecs() );
  TransVec          tvecs( refDetections.tvecs() );

  CalibrationDb calDb( opts.calibrationDb );
  if( !calDb.isOpened() ) {
    cerr << "Error opening calibration db: " << calDb.error().name() << endl;
    exit(-1);
  }

  kyotocabinet::HashDB results;
  if( !results.open( opts.resultsDb ) ) {
    cerr << "Error opening results db: " << results.error().name() << endl;
    exit(-1);
  }

      CalibrationReprojectFunctor func( objectPoints, imagePoints, rvecs, tvecs,  calDb, results );

  DB::Cursor *cur = calDb.cursor();
  cur->jump();
  const size_t chunk = 10;
  string key;
  vector< string > keys;
  while( cur->get_key( &key, true ) ) {
    keys.push_back( key );

    if( keys.size() >= chunk ) {
      func.setKeys( keys );
#ifdef USE_TBB
      tbb::parallel_for( tbb::blocked_range<size_t>(0, keys.size()), func );
#else
      func();
#endif
      keys.clear();
    }
  }

  func.setKeys( keys );
#ifdef USE_TBB
  tbb::parallel_for( tbb::blocked_range<size_t>(0, keys.size()), func );
#else
  func();
#endif

  delete cur;

  return 0;
}
