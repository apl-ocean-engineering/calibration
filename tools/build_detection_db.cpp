
#include <stdlib.h>
#include <getopt.h>
#include <time.h>

#include <iostream>
#include <iomanip>

#include <mutex>

#ifdef USE_TBB
#include "tbb/tbb.h"
#endif

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <kchashdb.h>

#include "board.h"
#include "detection.h"
#include "detection_db.h"
#include "file_utils.h"
#include "trendnet_time_code.h"
using namespace AplCam;

#include <tclap/CmdLine.h>
#include <glog/logging.h>
#include "apriltag_detector.h"
using namespace CameraCalibration;

using namespace std;
using namespace cv;
using kyotocabinet::HashDB;

namespace fs = boost::filesystem;

struct BuildDbOpts {
 public:
  BuildDbOpts()
      : seekTo(-1), intervalFrames(-1), waitKey( 1 ), 
      intervalSeconds( -1 ),
      boardFile(),
      benchmarkFile(),
      detectionDb(),
      doBenchmark( false ),
      doRewrite( false ),
      doDisplay( false ), yes( false ),
      doClahe( false ),
      mode
  {;}

  typedef enum { NONE, VIDEO, STILL } Mode;


  int seekTo, intervalFrames, waitKey;
  float intervalSeconds;
  string boardFile, benchmarkFile, detectionDb;
  bool doBenchmark, doRewrite, doDisplay, yes, doClahe;
  Mode mode;

  string inFile;

  const string boardPath( void )
  { return dataDir + "/boards/" + boardName + ".yml"; }

  const string tmpPath( const string &file )
  { return dataDir + "/tmp/" + file; }

  const string cachePath( const string &file = "" ) const
  { return dataDir + "/cache/" + file; }

  bool parseOpts( int argc, char **argv, stringstream &msg )
  {

    try {
      TCLAP::CmdLine cmd("build a database of detections", ' ', "0.1" );

      TCLAP::ValueArg<std::string> boardArg( "b", "board", "Board", true, "", "board name", cmd );
      TCLAP::ValueArg<std::string> detDbArg("D", "output-db", "Detection db", true, "", "db name", cmd );
      TCLAP::ValueArg<std::string> modeArg("m", "mode", "Mode", false, "video", "video or stills", cmd );

      TCLAP::ValueArg< int > waitKeyArg( "w", "wait-key", "Wait key", false, 0, "ms", cmd );

      TCLAP::SwitchArg doDisplayArg("X", "do-display", "Do display output", cmd );

      TCLAP::SwitchArg doClaheArg("C", "do-clahe", "Do CLAHE on images", cmd );

      TCLAP::UnlabeledMultiArg< std::string > fileNamesArg("files", "Files", "true", "file names", cmd );

      cmd.parse( argc, argv );

      inFiles = fileNamesArg.getValue();

      boardFile = boardArg.getValue();
      detectionDb = detDbArg.getValue();

      doClahe = doClaheArg.getValue();
      doDisplay = doDisplayArg.getValue();
      waitKey = waitKeyArg.getValue();

      string modeStr( modeArg.getValue() );
      if( modeStr == "still" )
        mode = STILL;
      else if( modeStr == "video" )
        mode = VIDEO;
      else {
        LOG(INFO) << "Mode not specified, assuming video";
        mode = VIDEO;
      }

    } catch( TCLAP::ArgException &e )
    {
      LOG(ERROR) << "Parsing error: " << e.error() << " for " << e.argId();
    }



    //      static struct option long_options[] = {
    //        { "seek-to", true, NULL, 's' },
    //        { "interval-frames", true, NULL, 'i' },
    //        { "interval-seconds", true, NULL, 'I' },
    //        { "do-rewrite", no_argument, NULL, 'R' },
    //        { "do-benchmark", required_argument, NULL, 'K' },

    return validate( msg );
  }

  bool validate( string &msg )
  {
    //    if( intervalFrames > 0 && intervalSeconds > 0 ) {
    //      msg = "Can't specify both interval frames and interval seconds at the same time.";
    //      return false;
    //    }

    if( mode  == NONE ) {
      LOG(ERROR) << "Mode not set (this shouldn't happen)";
      return false;
    }

    return true;
  }

  };




  class BuildDbMain 
  {
   public:
    BuildDbMain( BuildDbOpts &options )
        : opts( options ), _benchmark() 
    {;}

    ~BuildDbMain( void )
    {
      if( _benchmark.is_open() ) _benchmark.close();
    }

    int run( void ) {
      if( opts.doDisplay ) namedWindow( "BuildDb" );

      return doBuildDb();
    }

    int doBuildDb( void )
    {
      board = Board::load( opts.boardPath(), opts.boardName );
      if( !board ) {
        cerr << "Couldn't open board from " << opts.boardPath() << endl;
        return -1;
      }

      string videoSource( opts.inFile );
      VideoCapture vid( videoSource );
      if( !vid.isOpened() ) {
        cerr << "Couldn't open video source \"" << videoSource << "\"" << endl;
        return -1;
      }

      mkdir_p( opts.cachePath() );
      bool dbOpened;
      if( opts.detectionDb.empty() )
        dbOpened = db.open( opts.cachePath(), opts.inFile, true );
      else
        dbOpened = db.open( opts.detectionDb, true );

      if( !dbOpened ) {
        cerr << "Error opening database file: " << db.error().name() << endl;
        return -1;
      }

      double vidLength = vid.get( CV_CAP_PROP_FRAME_COUNT );

      db.setMeta( vidLength, 
                 vid.get( CV_CAP_PROP_FRAME_WIDTH ),
                 vid.get( CV_CAP_PROP_FRAME_HEIGHT ),
                 vid.get( CV_CAP_PROP_FPS ) );


      if( opts.intervalSeconds > 0 ) 
        opts.intervalFrames = opts.intervalSeconds * vid.get( CV_CAP_PROP_FPS );

      if( opts.seekTo > 0 )
        vid.set( CV_CAP_PROP_POS_FRAMES, opts.seekTo );

      FrameVec_t frames;
      const int chunkSize = 50;
      Mat img;

      while( vid.read( img ) ) {
        int currentFrame = vid.get( CV_CAP_PROP_POS_FRAMES );
        cout << currentFrame << " ";

        if(  !(opts.doDisplay || opts.doRewrite)  && db.has( currentFrame ) ) continue;

        Mat grey;

        if( opts.doClahe ) {

          //READ RGB color image and convert it to Lab
          cv::Mat lab_image;
          cv::cvtColor(img, lab_image, CV_BGR2Lab);

          // Extract the L channel
          std::vector<cv::Mat> lab_planes(3);
          cv::split(lab_image, lab_planes);  // now we have the L image in lab_planes[0]

          // apply the CLAHE algorithm to the L channel
          cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
          //clahe->setClipLimit(4);
          //clahe->setTileGridSize( 9 );
          cv::Mat dst;
          clahe->apply(lab_planes[0], dst);


          // Merge the the color planes back into an Lab image
          dst.copyTo(lab_planes[0]);
          cv::merge(lab_planes, lab_image);

          //// convert back to RGB
          //cv::Mat image_clahe;
          cv::cvtColor(lab_image, img, CV_Lab2BGR);

          //dst.copyTo( grey );
        } 

        cvtColor( img, grey, CV_BGR2GRAY );


        if( opts.doDisplay ) {
          Detection *detection = NULL;
          //cout << "Extracting from " << p.frame << ". ";

          vector<Point2f> pointbuf;

          detection = board->detectPattern( grey, pointbuf );

          // Trust the thread-safety of kyotocabinet
          db.save( currentFrame, *detection);

          detection->drawCorners( *board, img );

          imshow( "Detector", img );
          int ch = waitKey(1);
          switch( ch ) {
            case 'q': return 0;
          }


          delete detection;

        } else {
          if( !db.has( currentFrame ) || opts.doRewrite ) {
            frames.push_back( Frame( currentFrame, grey.clone() ) );
          }

          if( opts.intervalFrames > 1 ) {
            int destFrame = currentFrame + opts.intervalFrames - 1;

            if( destFrame < vidLength ) 
              vid.set( CV_CAP_PROP_POS_FRAMES, currentFrame + opts.intervalFrames - 1 );
            else 
              break;
          }

          if( frames.size() >= chunkSize ) {
            cout << endl;
            processFrames( frames );
          }
        }
      }

      cout << endl;
      processFrames( frames );


      return 0;
    }

    void processFrames( FrameVec_t &frames )
    {
      AprilTagDetectorFunctor f( frames, db, board );

#ifdef USE_TBB
      tbb::parallel_reduce( tbb::blocked_range<size_t>(0, frames.size()), f );
#else
      f();
#endif

      cout << "Processed " <<  frames.size() << " frames,  produced " << f.timingData.size() <<  " timing data" << endl;

      if( opts.doBenchmark ) saveTimingData( f.timingData );

      frames.clear();
    }

    void saveTimingData( const TimingDataVec_t &td )
    {
      if( !_benchmark.is_open() ) _benchmark.open( opts.benchmarkFile, ios_base::trunc );

      for( TimingDataVec_t::const_iterator itr = td.begin();  itr != td.end(); ++itr )
        _benchmark << (*itr).first << ',' << ((*itr).second / getTickFrequency()) << endl;
    }


   private:
    BuildDbOpts opts;

    DetectionDb db;
    Board *board;

    ofstream _benchmark;
  };


  class BuildDbVideo : public BuildDbMain {
   public:
    BuildDbVideo( BuildDbOpts &opts )
        : BuildDbMain( opts )
    {;}

  };

  class BuildDbStill : public BuildDbStill {
   public:
    BUildDBStill( BuildDbOpts &opts )
        : BuildDbMain( opts )
    {;}

  };



  int main( int argc, char **argv ) 
  {
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    BuildDbOpts opts;
    stringstream msg;
    if( !opts.parseOpts( argc, argv, msg ) ) {
      cout << msg << endl;
      exit(-1);
    }

    int retval = -1;
    if( opts.mode == BuildDbOpts::STILL ) {
      BuildDbMain main( opts );
      retval = main.run();
    } else {
      BuildDbVideo main( opts );
      retval = main.run();
    }

    exit( retval );

  }


