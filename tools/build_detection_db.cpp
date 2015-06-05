
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

#include <tclap/CmdLine.h>
#include <glog/logging.h>
#include <boost/filesystem.hpp>

#include <kchashdb.h>

#include "board.h"
#include "detection.h"
#include "detection_db.h"
#include "file_utils.h"
#include "trendnet_time_code.h"
using namespace AplCam;

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
      doDisplay( false ),
      doClahe( false ),
      hasCalledMakeDetectionDb(false)
  {;}



  int seekTo, intervalFrames, waitKey;
  float intervalSeconds;
  string boardFile, benchmarkFile, detectionDb;
  bool doBenchmark, doRewrite, doDisplay, yes, doClahe, noTbb;

  vector<string> inFiles;

  //  const string boardPath( void )
  //  { return dataDir + "/boards/" + boardName + ".yml"; }

  //  const string tmpPath( const string &file )
  //  { return dataDir + "/tmp/" + file; }
  //
  //  const string cachePath( const string &file = "" ) const
  //  { return dataDir + "/cache/" + file; }

  bool parseOpts( int argc, char **argv )
  {

    try {
      TCLAP::CmdLine cmd("build a database of detections", ' ', "0.1" );

      TCLAP::ValueArg<std::string> boardArg( "b", "board", "Board", true, "", "board name", cmd );
      TCLAP::ValueArg<std::string> detDbArg("D", "-tdb", "Detection db", true, "", "db name or directory", cmd );

      TCLAP::ValueArg< int > waitKeyArg( "w", "wait-key", "Wait key", false, 0, "ms", cmd );

      TCLAP::SwitchArg doDisplayArg("X", "do-display", "Do display output", cmd );

      TCLAP::SwitchArg doClaheArg("C", "do-clahe", "Do CLAHE on images", cmd );
      TCLAP::SwitchArg noTbbArg("T", "no-tbb", "Don't use TBB", cmd );

      TCLAP::UnlabeledMultiArg< std::string > fileNamesArg("files", "Files", "true", "file names", cmd );

      cmd.parse( argc, argv );

      inFiles = fileNamesArg.getValue();

      boardFile = boardArg.getValue();
      detectionDb = detDbArg.getValue();

      doClahe = doClaheArg.getValue();
      doDisplay = doDisplayArg.getValue();
      waitKey = waitKeyArg.getValue();
      noTbb = noTbbArg.getValue();

    } catch( TCLAP::ArgException &e ) {
      LOG(ERROR) << "Parsing error: " << e.error() << " for " << e.argId();
    }



    //      static struct option long_options[] = {
    //        { "seek-to", true, NULL, 's' },
    //        { "interval-frames", true, NULL, 'i' },
    //        { "interval-seconds", true, NULL, 'I' },
    //        { "do-rewrite", no_argument, NULL, 'R' },
    //        { "do-benchmark", required_argument, NULL, 'K' },

    return validate( );
  }

  bool validate( void )
  {
    //    if( intervalFrames > 0 && intervalSeconds > 0 ) {
    //      msg = "Can't specify both interval frames and interval seconds at the same time.";
    //      return false;
    //    }

    if( (inFiles.size() > 1 ) && (!fs::is_directory( detectionDb ) ) ) {
      LOG(ERROR) << "If multiple files are specified on command line, detection-db cannot be a file.";
      return false;
    }

    return true;
  }


  string makeDetectionDb( const string &inFile )
  {
    if( fs::path( detectionDb ).has_filename() && hasCalledMakeDetectionDb ) {
      LOG(ERROR) << "detection-db is a file, but makeDetectionDb called multiple times!";
      return detectionDb;
    }

    // Otherwise construct a path
    fs::path p( detectionDb );
    p += fs::path( inFile ).filename();
    p.replace_extension(".kch");

    hasCalledMakeDetectionDb = true;

    return p.string();
  }

  // Just insert a test 
  bool hasCalledMakeDetectionDb;

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
      board = Board::load( opts.boardFile,
                          fs::path( opts.boardFile ).stem().string() );
      if( !board ) {
        LOG(ERROR) << "Couldn't open board from " << opts.boardFile;
        return -1;
      }

      for( vector<string>::iterator itr = opts.inFiles.begin(); itr != opts.inFiles.end(); ++itr ) {

        string dbFile( opts.makeDetectionDb( *itr ) );

        DetectionDb db;

        if ( !db.open( dbFile, true ) )  {
          LOG(ERROR) << "Error opening database file " << dbFile << ": " << db.error().name();
          return -1;
        }

        if( isVideoFile( *itr ) ) {
          processVideo( *itr, db );
        } else {
          processStill( *itr, db );
        }

      }

      return 0;
    }

    bool isVideoFile( const string &filename )
    {
      // Not the most satisfying algorithm but good enough for now...
      boost::filesystem::path p(filename);
      string ext( p.extension().string() );

      if( ext == ".mov" || ext == ".mp4" || ext == ".avi" ) return true;

      return false;
    }

    virtual int processStill( const string &inFile, DetectionDb &db )
    {

      Mat img;
      img = imread( inFile );

      if( img.empty() ) {
        LOG(ERROR) << "Couldn't load image from \"" << inFile << "\"";
        return -1;
      }

      db.setMeta( 1, img.size().width, img.size().height, 1 );

      Detection *detection = processFrame( img, 0, db );
      doDisplay( img, detection );
      delete detection;

      return 0;
    }

    virtual int processVideo( const string &inFile, DetectionDb &db )
    {
      VideoCapture vid( inFile );
      if( !vid.isOpened() ) {
        cerr << "Couldn't open video source \"" << inFile << "\"" << endl;
        return -1;
      }

      double vidLength = vid.get( CV_CAP_PROP_FRAME_COUNT );

      db.setMeta( vidLength, 
                 vid.get( CV_CAP_PROP_FRAME_WIDTH ),
                 vid.get( CV_CAP_PROP_FRAME_HEIGHT ),
                 vid.get( CV_CAP_PROP_FPS ) );


      //      if( opts.intervalSeconds > 0 ) 
      //        opts.intervalFrames = opts.intervalSeconds * vid.get( CV_CAP_PROP_FPS );
      //
      //      if( opts.seekTo > 0 )
      //        vid.set( CV_CAP_PROP_POS_FRAMES, opts.seekTo );


      Mat img;

      if( opts.doDisplay ) {

        while( vid.read( img ) ) {
          int currentFrame = vid.get( CV_CAP_PROP_POS_FRAMES );
          Detection *detection = processFrame( img, currentFrame, db );
          if( !doDisplay( img, detection ) ) {
            delete detection;
            return 0;
          }

          delete detection;
        }


      } else {
        FrameVec_t frames;
        const int chunkSize = 50;
        int currentFrame;

        while( vid.read( img ) ) {
          currentFrame = vid.get( CV_CAP_PROP_POS_FRAMES );
          //cout << currentFrame << " ";

          if( !opts.doRewrite  && db.has( currentFrame ) ) continue;

          frames.push_back( Frame( currentFrame, img.clone() ) );

          //          if( opts.intervalFrames > 1 ) {
          //            int destFrame = currentFrame + opts.intervalFrames - 1;
          //
          //            if( destFrame < vidLength ) 
          //              vid.set( CV_CAP_PROP_POS_FRAMES, currentFrame + opts.intervalFrames - 1 );
          //            else 
          //              break;
          //          }

          if( frames.size() >= chunkSize ) {
            LOG(INFO) << "Processing up to frame " << currentFrame;
            bulkProcessFrames( frames, db, opts.noTbb );
          }
        }

        LOG(INFO) << "Processing remaining frames to " << currentFrame;
        bulkProcessFrames( frames, db, opts.noTbb );

      }

      return 0;
    }


    void prepFrame( Mat &frame )
    {

      if( opts.doClahe ) {

        //READ RGB color image and convert it to Lab
        cv::Mat lab_image;
        cv::cvtColor(frame, lab_image, CV_BGR2Lab);

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
        cv::cvtColor(lab_image, frame, CV_Lab2BGR);

        //dst.copyTo( grey );
      } 

      //cvtColor( frame, frame, CV_BGR2GRAY );
    }

    Detection *processFrame( Mat &frame, int currentFrame, DetectionDb &db )
    {
      prepFrame( frame );

      Detection *detection = board->detectPattern( frame );
      db.save( currentFrame, *detection);

      return detection;
    }


    void bulkProcessFrames( FrameVec_t &frames, DetectionDb &db, bool noTbb )
    {
      AprilTagDetectorFunctor f( frames, db, board );

      for( FrameVec_t::iterator itr = frames.begin(); itr != frames.end(); ++itr ) 
        prepFrame( itr->img );

#ifdef USE_TBB
      if( !noTbb )
        tbb::parallel_reduce( tbb::blocked_range<size_t>(0, frames.size()), f );
      else
#else
        f();
#endif

      cout << "Processed " <<  frames.size() << " frames,  produced " << f.timingData.size() <<  " timing data" << endl;

      if( opts.doBenchmark ) saveTimingData( f.timingData );

      frames.clear();
    }

    bool doDisplay( Mat &img, Detection *detection )
    {
      detection->drawCorners( *board, img );

      imshow( WindowName, img );

      int ch = waitKey( opts.waitKey );

      switch( ch ) {
        case 'q': return false;
      }

    }



    void saveTimingData( const TimingDataVec_t &td )
    {
      if( !_benchmark.is_open() ) _benchmark.open( opts.benchmarkFile, ios_base::trunc );

      for( TimingDataVec_t::const_iterator itr = td.begin();  itr != td.end(); ++itr )
        _benchmark << (*itr).first << ',' << ((*itr).second / getTickFrequency()) << endl;
    }


   protected: 
    BuildDbOpts opts;

    Board *board;

    ofstream _benchmark;

    const string WindowName = "BuildDb";
  };




  int main( int argc, char **argv ) 
  {
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    BuildDbOpts opts;
    if( !opts.parseOpts( argc, argv ) ) {
      exit(-1);
    }

    int retval;
    BuildDbMain main( opts );
    retval = main.run();

    exit( retval );

  }


