
#include <stdlib.h>
#include <getopt.h>
#include <time.h>

#include <iostream>
#include <iomanip>

#include <mutex>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <kchashdb.h>
#include <tclap/CmdLine.h>
#include <glog/logging.h>

#include "board.h"
#include "detection/detection.h"
#include "detection_db.h"
#include "file_utils.h"
#include "trendnet_time_code.h"

#include "video_prefs.h"

#include "composite_canvas.h"
using namespace AplCam;

#include "apriltag_detector.h"
using namespace CameraCalibration;

using namespace std;
using namespace cv;
using kyotocabinet::HashDB;

struct BuildDbOpts {
  public:
    BuildDbOpts()
      : boardName(),
      detectionDbDir(),
      doDisplay( false ),
      doParallel( false ),
      doRewrite( false )
  {;}


    int waitKey;
    float intervalSeconds;
    string boardName, dataDir, detectionDbDir, doAnnotate;
    bool doDisplay, doParallel, doRewrite;

    string inFile;

    const string boardPath( void )
    { return dataDir + "/boards/" + boardName + ".yml"; }

    const string tmpPath( const string &file )
    { return dataDir + "/tmp/" + file; }

    const string cachePath( const string &file = "" ) const
    { return dataDir + "/cache/" + file; }

    const string detectionDbPath( const int i )
    {
      return detectionDbDir + "/" + ( ( i == 0 ) ? "zero.kch" : "one.kch" );
    }

    bool parseOpts( int argc, char **argv, stringstream &msg )
    {

      try {

        TCLAP::CmdLine cmd("build_stereo_detection_db", ' ', "0.1");

        TCLAP::SwitchArg doDisplayArg("X", "do-display", "display", cmd, false );
        TCLAP::ValueArg<std::string> dataDirArg( "d", "data-dir", "Data directory", false, "data/", "dir", cmd );
        TCLAP::ValueArg<std::string> detectionDbDirArg( "", "detection-db-dir", "Detection db directory", false, ".", "dir", cmd );
        TCLAP::ValueArg<std::string> boardNameArg( "b", "board-name", "Board name", true, "", "dir", cmd );

        TCLAP::SwitchArg doParallelArg("P", "do-parallel", "Do run in parallel", cmd, false );
        TCLAP::SwitchArg doRewriteArg("R", "do-rewrite", "Do rewrite existing entries in detection d/b", cmd, false );
        TCLAP::ValueArg<std::string> doAnnotateArg("N", "do-annotate", "Do annotate entries into a video", false, "", "filename", cmd );

        TCLAP::UnlabeledValueArg< std::string > videoFileArg( "video-file", "Video file", true, "", "file name", cmd );

        cmd.parse( argc, argv );

        inFile = videoFileArg.getValue();
        dataDir = dataDirArg.getValue();
        detectionDbDir = detectionDbDirArg.getValue();
        doDisplay = doDisplayArg.getValue();
        doParallel = doParallelArg.getValue();
        doRewrite = doRewriteArg.getValue();
        doAnnotate = boost::filesystem::path( doAnnotateArg.getValue() ).replace_extension( VideoExtension ).string();

        boardName = boardNameArg.getValue();


      } catch( TCLAP::ArgException &e ) {
        LOG( ERROR ) << "error: " << e.error() << " for arg " << e.argId();
      }

      return validate( msg );
    }

    bool validate( stringstream &msg )
    {
      if( !file_exists( boardPath() ) ) {
        msg << "Can't find board file: " << boardPath();
        return false;
      }

      if( !directory_exists( detectionDbDir ) ) {
        msg << "Detection db directory doesn't exist: " << detectionDbDir;
        return false;
      }

      return true;
    }

};




class BuildDbMain
{
  public:
    BuildDbMain( BuildDbOpts &options )
      : opts( options )
    {;}

    ~BuildDbMain( void )
    {
    }

    int run( void ) {
      board = Board::load( opts.boardPath(), opts.boardName );
      if( !board ) {
        LOG(ERROR) << "Couldn't open board from " << opts.boardPath() << endl;
        return -1;
      }

      CompositeVideo compVid( opts.inFile );
      if( !compVid.isOpened() ) {
        LOG(ERROR) << "Couldn't open video source \"" << opts.inFile << "\"" << endl;
        return -1;
      }

      for( int i = 0; i < 2; ++i ) {
        bool dbOpened;
        dbOpened = db[i].open( opts.detectionDbPath(i), true );

        if( !dbOpened ) {
          LOG(ERROR) << "Error opening database file " << opts.detectionDbPath(i) << ": " << db[i].error().name() << endl;
          return -1;
        }

        db[i].setMeta( compVid.get( CV_CAP_PROP_FRAME_COUNT ),
            compVid.get( CV_CAP_PROP_FRAME_WIDTH )/ 2.0,
            compVid.get( CV_CAP_PROP_FRAME_HEIGHT ),
            compVid.fps() );
      }

      int retval = 0;
      if( opts.doDisplay ) {
        namedWindow( BuildDbWindowName );
        return doDisplay( compVid );
      } else if( opts.doParallel ) {
        LOG(INFO) << "Processing frames in parallel.";
        retval = doParallel( compVid );
      } else {
        retval = doSingular( compVid );
      }

        if( opts.doAnnotate.length() > 0 ) {
          LOG(INFO) << "Annotating";

          compVid.rewind();
          doAnnotate( compVid );
        }

        return retval;

    }

    int doSingular( CompositeVideo &compVid )
    {

      CompositeCanvas canvas;

      while( compVid.read( canvas ) ) {
        int currentFrame = compVid.get( CV_CAP_PROP_POS_FRAMES );

        for( int i = 0; i < 2; ++i ) {

          if( db[i].has( currentFrame ) ) continue;

          //Mat gray;
          //cvtColor( canvas[i], gray, COLOR_BGR2GRAY );

          Detection *det = board->detectPattern( canvas[i] );

          db[i].save( currentFrame, *det );
          delete det;
        }

      }

      return 0;
    }


    int doParallel( CompositeVideo &compVid )
    {
      CompositeCanvas canvas;

      const int chunkSize = 50;
      FrameVec_t frames[2];

      while( compVid.read( canvas ) ) {
        int currentFrame = compVid.get( CV_CAP_PROP_POS_FRAMES );

        for( int i = 0; i < 2; ++i ) {

          if( !db[i].has( currentFrame ) || opts.doRewrite ) {
            frames[i].push_back( Frame( currentFrame, canvas[i].clone() ) );
          }
        }

        if( frames[0].size() >= chunkSize || frames[1].size() >= chunkSize ) {
          processFrames( frames );
        }

      }

      processFrames( frames );

      return 0;
    }

    void processFrames( FrameVec_t *frames )
    {
      for( size_t i = 0; i < 2; ++i ) {
        AprilTagDetectorFunctor f( frames[i], db[i], board );

#ifdef USE_TBB
        LOG(INFO) << "Processing with TBB" << endl;
        tbb::parallel_for( tbb::blocked_range<size_t>(0, frames[i].size()), f );
#else
        LOG(INFO) << "Processing without TBB" << endl;
        f();
#endif

        LOG(INFO) << "Processed " <<  frames[i].size() << " frames" << endl;

        frames[i].clear();
      }
    }

    int doDisplay( CompositeVideo &compVid )
    {

      //double vidLength = compVid.get( CV_CAP_PROP_FRAME_COUNT );
      double fps = compVid.get( CV_CAP_PROP_FPS );

      int wait = 1.0/fps * 1000, wk = wait;
      LOG(INFO) << " Wait = " << wait;

      CompositeCanvas canvas;

      while( compVid.read( canvas ) ) {
        int currentFrame = compVid.get( CV_CAP_PROP_POS_FRAMES );

        for( int i = 0; i < 2; ++i ) {

          if( db[i].has( currentFrame ) && !opts.doRewrite ) continue;

          //Mat gray;
          //cvtColor( canvas[i], gray, COLOR_BGR2GRAY );

          Detection *det = board->detectPattern( canvas[i] );
          cout << det->corners.size() << " ";

          det->drawCorners( *board, canvas[i] );

          db[i].save( currentFrame, *det );
          delete det;
        }

        imshow( BuildDbWindowName, canvas );
        int ch = waitKey( wk );
        if( ch == ' ' ) {
          wk = (wk == 0) ? wait : 0;
        } else if( ch == 'q') {
          return 0;
        }

      }

      return 0;
    }

    int doAnnotate( CompositeVideo &compVid )
    {
      CompositeCanvas canvas;

      VideoWriter writer( opts.doAnnotate, VideoCodec, 30, compVid.fullSize() );


      while( compVid.read( canvas ) ) {
        int currentFrame = compVid.get( CV_CAP_PROP_POS_FRAMES );

        if( (currentFrame % 100) == 0 )
          LOG(INFO) << currentFrame;

        for( int i = 0; i < 2; ++i ) {

          if(  !db[i].has( currentFrame ) ) continue;

          Detection *det = db[i].load( currentFrame );

          det->drawCorners( *board, canvas[i] );

          delete det;
        }

        writer.write( canvas );

      }

      return 0;
    }

  private:

    static const string BuildDbWindowName;

    BuildDbOpts opts;

    DetectionDb db[2];
    Board *board;
};

const string BuildDbMain::BuildDbWindowName = "BuildStereoDb";


int main( int argc, char **argv )
{
  google::InitGoogleLogging( argv[0] );
  FLAGS_logtostderr = 1;

  BuildDbOpts opts;
  stringstream msg;
  if( !opts.parseOpts( argc, argv, msg ) ) {
    cout << msg.str() << endl;
    exit(-1);
  }

  BuildDbMain main( opts );

  exit( main.run() );

}
