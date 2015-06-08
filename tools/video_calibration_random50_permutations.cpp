#include <cctype>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <getopt.h>

#include <iostream>

#include "glog/logging.h"

#include "file_utils.h"
#include "board.h"
#include "detection.h"
#include "detection_set.h"
#include "image.h"

#include "distortion_model.h"
using namespace Distortion;

#include "calib_frame_selectors/calib_frame_selector_opts.h"
#include "calib_frame_selectors/calib_frame_selectors.h"
using namespace AplCam::CalibFrameSelectors;

#include "calibration_permutations.h"

using namespace cv;
using namespace std;

using kyotocabinet::HashDB;
using kyotocabinet::DB;

int main( int argc, char** argv )
{
  google::InitGoogleLogging("video_calibration_permutation");
  FLAGS_logtostderr = 1;

  CalibrationOpts opts;

  if( !opts.parseOpts( argc, argv ) ) {
    exit(-1);
  }

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
  const int minImages = 50;
  const int maxImages = 50;
  const int maxReps = 100;

  keys.clear();
  calDb.findKeysStartingWith( "random", keys );

  // For simplicity, just configure in code.
  for( int i = minImages; i < maxImages; i+= spacing ) {

    int count = std::max( minImages, i );
    int maxSets = std::min( maxReps, (maxImages-count) );

    // Parse out the existing keys with the correct length
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

    // Now run them all
    CalibrateFunctor func( opts, db.imageSize(), calDb, detSets );
    func();

    // Delete all detection sets
    for( size_t j = 0; j < detSets.size(); ++j ) delete detSets[j];
    detSets.clear();

  }

  //  Clean up anything that may be left
  CalibrateFunctor func( opts, imageSize, calDb, detSets );
  func();

  return 0;
}
