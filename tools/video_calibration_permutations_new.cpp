#include <cctype>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <getopt.h>

#include <iostream>

#include "glog/logging.h"

#include <kchashdb.h>

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

#include "calibration_opts.h"
#include "calibrator.h"
#include "permutation_helpers.h"

using namespace cv;
using namespace std;

using kyotocabinet::HashDB;
using kyotocabinet::DB;


class PermutationOpts : public CalibrationOpts {

  public:

    PermutationOpts() :
      CalibrationOpts()
  {;}

    int randomStart, randomEnd, randomStep, randomCount, minTags;
    bool doAll, doRewrite;

  protected:

    virtual void doParseCmdLine( TCLAP::CmdLine &cmd, int argc, char **argv )
    {
      TCLAP::ValueArg< int > randomStartArg("", "random-start", "Random start", false, 0, "Random start", cmd );
      TCLAP::ValueArg< int > randomEndArg("", "random-end", "Random end", false, INT_MAX, "Random end", cmd );
      TCLAP::ValueArg< int > randomStepArg("", "random-step", "Random step", false, 50, "Random step", cmd );
      TCLAP::ValueArg< int > randomCountArg("", "random-count", "Random count", false, 0, "Random count", cmd );
      TCLAP::ValueArg< int > minTagsArg("", "min-tags", "Minimum tags", false, -1, "Minimum tags", cmd );

      TCLAP::SwitchArg doAllArg("", "do-all", "Do All", cmd, false );
      TCLAP::SwitchArg doRewriteArg("", "do-rewrite", "Do rewrite", cmd, false );

      CalibrationOpts::doParseCmdLine( cmd, argc, argv );

      randomStart = randomStartArg.getValue();
      randomEnd   = randomEndArg.getValue();
      randomStep  = randomStepArg.getValue();
      randomCount = randomCountArg.getValue();

      minTags     = minTagsArg.getValue();

      doAll       = doAllArg.getValue();
      doRewrite   = doRewriteArg.getValue();
    }

    virtual bool validate( void )
    {
      if( !calibrationFile.empty() ) {
        LOG(ERROR) << "Can't set a calibration file when doing permutations.";
        return false;
      }

      if( calibrationDb.empty() ) {
        LOG(ERROR) << "Calibration db not specified.";
      }

      return true;
    }

};

class PermutationMain {
  public:

    PermutationMain( PermutationOpts &opts_ )
      : opts( opts_ )
    {;}

    int run( void )
    {

      if( ! db.open( opts.detectionDb,  false ) ) {
        cerr << "Error opening detection db \"" << opts.detectionDb << "\". Error: " << db.error().name() << endl;
        return -1;
      }

//      Size imageSize = db.imageSize();

      if( opts.doRewrite ) {
        // Simplest thing to do is just truncate the existing file
        kyotocabinet::HashDB h;
        h.open( opts.calibrationDb, HashDB::OWRITER | HashDB::OTRUNCATE );
        h.close();

      }

      if( !calDb.open( opts.calibrationDb, true ) ) {
        cerr << "Error opening calibration db \"" << opts.calibrationDb << "\".  Error: " << endl;
        exit(-1);
      }


      if( opts.doAll ) queueAll();

      // == Random ==
      queueRandom();

      // Final cleanup
      processDetSets();

      return 0;
    }


    void queueAll( void )
    {
      vector<string> keys;
      calDb.findKeysStartingWith( "all", keys );
      if( (keys.size() == 0) ) {
        DetectionSet *all = new DetectionSet;
        AllFrameSelector( opts.minTags ).generate( db, *all );
        pushDetSet( all );
      }
    }

    void queueRandom( void )
    {
      vector<string> keys;
      keys.clear();
      calDb.findKeysStartingWith( "random", keys );

      const int minImages = 10;
      int stopAt = std::min( opts.randomEnd, db.vidLength()-1 );

      for( int i = opts.randomStart; i <= stopAt; i+= opts.randomStep ) {

        int count = std::max( minImages, i );
        int maxSets = std::min( opts.randomCount, (db.vidLength()-count) );

        // Parse out the existing keys with the correct length
        int existing = std::count_if( keys.begin(), keys.end(), RandomKeyCounter( count ) );

        size_t todo = (maxSets > existing) ? (maxSets - existing) : 0;

        cout << "For random set of " << count << " images, found " << existing << " sets, must do " << todo << endl;

        if( todo == 0 ) continue;

        size_t j = 0;
        while( j < todo ) {
          DetectionSet *detSet = new DetectionSet;
          RandomFrameSelector( count, opts.minTags ).generate( db, *detSet );

          if( calDb.has( detSet->name() ) ) continue;

          ++j;
          pushDetSet( detSet );
        }

      }
    }

    void pushDetSet( DetectionSet *detSet )
    {
      detSets.push_back( detSet );

      if( detSets.size() > 50 )  processDetSets();
    }

    void processDetSets( void )
    {
      // Now run them all
      CalibrateFunctor func( opts, db.imageSize(), calDb, detSets );
      func();

      // Delete all detection sets
      for( size_t j = 0; j < detSets.size(); ++j ) delete detSets[j];
      detSets.clear();
    }

  protected:

    PermutationOpts &opts;
    DetectionDb db;
    CalibrationDb calDb;
    vector< DetectionSet * > detSets;

};


int main( int argc, char** argv )
{
  google::InitGoogleLogging("video_calibration_permutation");
  FLAGS_logtostderr = 1;

  PermutationOpts opts;

  if( !opts.parseOpts( argc, argv ) )  exit(-1);

  PermutationMain perm( opts );
  return perm.run();
}

