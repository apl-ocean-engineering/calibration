#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <cctype>
#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <time.h>
#include <getopt.h>

#include <iostream>

#include <glog/logging.h>

#include "file_utils.h"
#include "board.h"
#include "detection.h"
#include "detection_db.h"
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

class VideoCalibrationOpts : public CalibrationOpts {

  public:

    VideoCalibrationOpts()
      : CalibrationOpts(),
      videoFile()
  {;}

    string videoFile;

    AplCam::CalibFrameSelectors::Type_t selector;

    IntervalSelectorOpts intervalSelectorOpts;
    RandomSelectorOpts randomSelectorOpts;
    KeyframeSelectorOpts keyframeSelectorOpts;

  protected:

    virtual void doParseCmdLine( TCLAP::CmdLine &cmd, int argc, char **argv )
    {
      TCLAP::UnlabeledValueArg< std::string > selectorArg( "selector", "Frame selector", true, "", "{all|interval|random}", cmd );
      TCLAP::UnlabeledValueArg< std::string > videoFileArg( "video-file", "Video file", true, "", "Video file", cmd );

      // Options for selectors
      TCLAP::ValueArg< int > randomCountArg( "c", "count", "Count", false, 1, "Count", cmd );

      CalibrationOpts::doParseCmdLine( cmd, argc, argv );

      videoFile = videoFileArg.getValue();

      string selector( selectorArg.getValue() );
      if( selector.compare("random") == 0 ) {
        selector = SPLIT_RANDOM;
        randomSelectorOpts.count = randomCountArg.getValue();
      } else {
        LOG(ERROR) << "Don't understand the selector \"" << selector << "\"";
      }

      // Hm, need to restructure the verb parsing
      // Next, expect a verb
      //char *verb = argv[ optind++ ];
      //bool success = false;
      //if( !strcasecmp( verb, "all" ) ) {
      //  selector = SPLIT_ALL;
      //  success = true;
      //} else if( !strcasecmp( verb, "random" ) ) {
      //  selector = SPLIT_RANDOM;
      //  success = randomSelectorOpts.parseOpts( argc, argv, msg );
      //} else if( !strcasecmp( verb, "interval" ) ) {
      //  selector = SPLIT_INTERVAL;
      //  success = intervalSelectorOpts.parseOpts( argc, argv, msg );
      //} else if( !strcasecmp( verb, "keyframe" ) ) {
      //  selector = SPLIT_KEYFRAME;
      //  success = keyframeSelectorOpts.parseOpts( argc, argv, msg );
      //} else {
      //  msgstrm << "Don't understand verb \"" << verb << "\"";
      //  msg = msgstrm.str();
      //  return false;
      //}

    }

    virtual bool validate( void )
    {

      if( !file_exists( videoFile ) ) {
        LOG(ERROR) << "Can't open video file " << videoFile << endl;
        return false;
      }

      if( !calibrationDb.empty() ) {
        if( !calibrationFile.empty() ) {
          LOG(ERROR) << "Can't set both calibration file and calibration db";
          return false;
        }
      }

      // The super will auto-fill calibrationFile if not set
      if( !calibrationDb.empty() ) calibrationFile.clear();

      return true;
    }

};




int main( int argc, char** argv )
{
  google::InitGoogleLogging( argv[0] );
  FLAGS_logtostderr = 1;


  VideoCalibrationOpts opts;

  if( !opts.parseOpts( argc, argv ) ) exit(-1);


  DetectionDb db;
  Size imageSize;

  if( opts.detectionDb.empty() ) {
    if( ! db.open( opts.cachePath(), opts.videoFile ) ) {
      LOG(ERROR) << "Error opening db error: " << db.error().name();
      return -1;
    }

    string videoSource( opts.videoFile );
    VideoCapture vid( videoSource );
    if( !vid.isOpened() ) {
      LOG(ERROR) << "Couldn't open video source \"" << videoSource << "\"";
      return -1;
    }
    //int vidLength = vid.get( CV_CAP_PROP_FRAME_COUNT );

    // Get image size
    imageSize = Size( vid.get( CV_CAP_PROP_FRAME_WIDTH ), vid.get(CV_CAP_PROP_FRAME_HEIGHT ) );
  } else {
    if( ! db.open( opts.detectionDb ) ) {
      LOG(ERROR) << "Error opening db error: " << db.error().name();
      return -1;
    }

    if( ! db.has_meta() ) {
      LOG(ERROR) << "Database doesn't have metainformation";
      return -1;
    }

    imageSize = db.imageSize();
  }

  Board *board = Board::load( opts.boardPath(), opts.boardName );

  DetectionSet detSet;
  switch( opts.selector ) {
    case CalibFrameSelectors::SPLIT_ALL:
      AllFrameSelector().generate( db, detSet );
      break;
    case CalibFrameSelectors::SPLIT_RANDOM:
      RandomFrameSelector( opts.randomSelectorOpts ).generate( db, detSet );
      break;
    case CalibFrameSelectors::SPLIT_INTERVAL:
      IntervalFrameSelector( opts.intervalSelectorOpts ).generate( db, detSet );
      break;
    case CalibFrameSelectors::SPLIT_KEYFRAME:
      KeyframeFrameSelector( *board, opts.keyframeSelectorOpts ).generate( db, detSet );
      break;
    default:
      LOG(ERROR) << "Unknown frame selector.";
      exit(-1);
  }

  if( opts.doValidate ) {
    LOG(INFO) << "Validating detection set.";
    size_t sizeBefore = detSet.size();
    detSet.validate();
    size_t sizeAfter = detSet.size();

    LOG(INFO) << "Detection set went from " << sizeBefore << " to " << sizeAfter << " during validation.";
  }

  Calibrator cal( opts, detSet, imageSize );
  cal.run();

  if( cal.result.good ) {

    if( !opts.calibrationDb.empty() ) 
      cal.saveDb( opts.calibrationDb, opts.overwriteDb );
    else if( !opts.calibrationFile.empty() ) 
      cal.saveFile( opts.calibrationFile );


    if( opts.saveBoardPoses.length() > 0 ) {
      DetectionDb savedPoses( opts.saveBoardPoses, true ); 
      cal.updateDetectionPoses( detSet );
      savedPoses.save( detSet );
    }
  } else {
    cout << "Calibration failed." << endl;
  }

  return 0;
}
