#include <cctype>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <getopt.h>

#include <iostream>

// Disable TBB temporarily, assume threading in the calibration routine (Ceres) itself.
#undef USE_TBB

#ifdef USE_TBB
#include "tbb/tbb.h"
using namespace tbb;
#endif

#include "glog/logging.h"

#include "file_utils.h"
#include "board.h"
#include "detection.h"
#include "detection_set.h"
#include "image.h"

#include "distortion_model.h"
using namespace Distortion;

#include "calibration_db.h"
#include "calibrator.h"
using namespace AplCam;

#include "calib_frame_selectors/calib_frame_selector_opts.h"
#include "calib_frame_selectors/calib_frame_selectors.h"
using namespace AplCam::CalibFrameSelectors;

#include "calibration_opts.h"

using namespace cv;
using namespace std;

using kyotocabinet::HashDB;
using kyotocabinet::DB;

int main( int argc, char** argv )
{
  google::InitGoogleLogging("video_calibration_permutation");

  CalibrationOpts opts;

  if( !opts.parseOpts( argc, argv ) ) exit(-1);

  DetectionDb db;
  if( ! db.open( opts.detectionDb,  false ) ) {
    cerr << "Error opening detection db \"" << opts.detectionDb << "\". Error: " << db.error().name() << endl;
    return -1;
  }

  Size imageSize = db.imageSize();

  CalibrationDb calDb( opts.calibrationDb );
  if( !calDb.isOpened() ) {
    cerr << "Error opening calibration db \"" << opts.calibrationDb << "\".  Error: " << endl;
    exit(-1);
  }


  // == Handle all ==
  vector< DetectionSet * > detSets;

  vector<string> keys;
  calDb.findKeysStartingWith( "all", keys );
  if( keys.size() == 0 ) {
    DetectionSet *all = new DetectionSet;
    AllFrameSelector().generate( db, *all );
    detSets.push_back( all );
  }

  // == Random ==

  const int spacing = 100;
  const int minImages = 10;
  const int maxReps = 10;
  const int maxImages = db.vidLength();

  keys.clear();
  calDb.findKeysStartingWith( "random", keys );

  // For simplicity, just configure in code.
  for( int i = 0; i < maxImages; i+= spacing ) {

    int count = std::max( minImages, i );
    int maxSets = std::min( maxReps, (maxImages-count) );


    // Parse out the keys with the correct length
    int existing = std::count_if( keys.begin(), keys.end(), RandomKeyCounter( count ) );

    size_t todo = (maxSets > existing) ? (maxSets - existing) : 0;


    cout << "For random set of " << count << " images, found " << existing << " sets, must do " << todo << endl;

    if( todo == 0 ) continue;

    for( size_t j = 0; detSets.size() < todo; ++j ) {
      DetectionSet *detSet = new DetectionSet;
      RandomFrameSelector( count ).generate( db, *detSet );

      if( calDb.has( detSet->name() ) ) continue;

      detSets.push_back( detSet );
    }


    // Now run each one
    CalibrateFunctor func( opts, db.imageSize(), calDb, detSets );
#ifdef USE_TBB
    parallel_for( blocked_range<size_t>(0,detSets.size()), func );
#else
    func();
#endif

    // Delete all detection sets
    for( size_t j = 0; j < detSets.size(); ++j ) delete detSets[j];
    detSets.clear();

  }

  // == Interval ==
  keys.clear();
  calDb.findKeysStartingWith( "interval", keys );

  const int minInterval = 2;
  const int maxInterval = 150;
  const int intervalSpacing = 10;
  const int intervalReps = 10;

  for( int i = 0; i <= maxInterval; i += intervalSpacing ) {
    int interval = std::max( i, minInterval );
    int reps = std::min( interval, intervalReps );
    float deltaOffset = interval / reps;

    for( int j = 0; j < reps; ++j ) {
      int offset = round( deltaOffset * j );
      offset = min( offset, interval-1 );

      cout << "Want to try every " << interval << " frames starting with " << offset << endl;

      if( std::find_if( keys.begin(), keys.end(), IntervalKeyFinder( offset, interval ) ) != keys.end() ) {
        cout  << "Key already exists" << endl;
      } else {
        DetectionSet *detSet = new DetectionSet;
        IntervalFrameSelector( offset, interval ).generate( db, *detSet );

        if( calDb.has( detSet->name() ) ) continue;

        detSets.push_back( detSet );
      }
    }

    // Now run each one
    CalibrateFunctor func( opts, imageSize, calDb, detSets );
#ifdef USE_TBB
    parallel_for( blocked_range<size_t>(0,detSets.size()), func );
#else
    func();
#endif

    // Delete all detection sets
    for( size_t j = 0; j < detSets.size(); ++j ) delete detSets[j];
    detSets.clear();

  }

  //  Clean up anything that may be left
  CalibrateFunctor func( opts, imageSize, calDb, detSets );
#ifdef USE_TBB
  parallel_for( blocked_range<size_t>(0,detSets.size()), func );
#else
  func();
#endif


  return 0;
}
