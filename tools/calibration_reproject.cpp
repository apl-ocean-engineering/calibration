#include <cctype>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <getopt.h>

#include <iostream>

#include <glog/logging.h>

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
#include "calibration_opts.h"
#include "calibrator.h"
using namespace AplCam;

#include "calib_frame_selectors/calib_frame_selectors.h"
using namespace AplCam::CalibFrameSelectors;


using namespace cv;
using namespace std;

using kyotocabinet::HashDB;
using kyotocabinet::DB;

class CalibrationReprojectOpts : public CalibrationOpts {

  public:

    CalibrationReprojectOpts()
      : CalibrationOpts(), 
      resultsDb(), 
      referenceDb()
  {;}

    string resultsDb;
    string referenceDb;

  protected:


    virtual void doParseCmdLine( TCLAP::CmdLine &cmd, int argc, char **argv )
    {
      TCLAP::ValueArg< std::string > resultsDbArg("R", "results-db", "Results db", true, "", "db name", cmd );
      TCLAP::ValueArg< std::string > referenceDbArg("r", "reference-db", "Reference db", true, "", "db name", cmd );

      CalibrationOpts::doParseCmdLine( cmd, argc, argv );

      resultsDb = resultsDbArg.getValue();
      referenceDb = referenceDbArg.getValue();
    }

    virtual bool validate( void )
    {

      if( !checkForDb( calibrationDb, "calibration" ) ) return false;
      if( !checkForDb( referenceDb, "reference" ) ) return false;

      if( resultsDb.empty() ) {
         LOG(ERROR) << "Must specify results db";
        return false;
      }

      return true;
    }

    bool checkForDb( const string &db, const string &name )
    {
      if( !file_exists( db ) ) {
        LOG(ERROR) << "Can't find " << name << " db \"" << db << "\"";
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


      FileStorage fs( value, FileStorage::READ | FileStorage::MEMORY );
      DistortionModel *distModel = CameraFactory::Unserialize( fs );

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
      result.good = ((int)fs["success"] == 1);

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

  if( !opts.parseOpts( argc, argv ) ) exit(-1);

  DetectionDb refDets;
  if( ! refDets.open( opts.referenceDb ) ) {
    cerr << "Error opening reference detection db: " << refDets.error().name() << endl;
    exit(-1);
  }

  DetectionSet refDetections;
  AllGoodFrameSelector vs;
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
