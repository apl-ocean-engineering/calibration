
#include <stdlib.h>
#include <getopt.h>
#include <time.h>

#include <iostream>
#include <iomanip>

#include <mutex>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <kchashdb.h>

#include <TCLAP/CmdLine.h>

#include <glog/logging.h>

#include "board.h"
#include "detection.h"
#include "detection_db.h"
#include "file_utils.h"
#include "trendnet_time_code.h"

#include "composite_canvas.h"
using namespace AplCam;

using namespace std;
using namespace cv;
using kyotocabinet::HashDB;

struct BuildDbOpts {
  public:
    BuildDbOpts()
      : boardName(), 
      detectionDbDir(),
      doDisplay( false )
  {;}


    int waitKey;
    float intervalSeconds;
    string boardName, dataDir, detectionDbDir;
    bool doDisplay;

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

        TCLAP::UnlabeledValueArg< std::string > videoFileArg( "video-file", "Video file", true, "", "file name", cmd );

        cmd.parse( argc, argv );

        inFile = videoFileArg.getValue();
        dataDir = dataDirArg.getValue();
        detectionDbDir = detectionDbDirArg.getValue();
        doDisplay = doDisplayArg.getValue();
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
      if( opts.doDisplay ) namedWindow( BuildDbWindowName );

      return doBuildDb();
    }

    int doBuildDb( void )
    {
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
      }

      double vidLength = compVid.get( CV_CAP_PROP_FRAME_COUNT );
      double fps = compVid.get( CV_CAP_PROP_FPS );

      int wait = 1.0/fps * 1000, wk = wait;

      //     db.setMeta( vidLength, 
      //                 vid.get( CV_CAP_PROP_FRAME_WIDTH ),
      //                 vid.get( CV_CAP_PROP_FRAME_HEIGHT ),
      //                 vid.get( CV_CAP_PROP_FPS ) );


      //     if( opts.intervalSeconds > 0 ) 
      //       opts.intervalFrames = opts.intervalSeconds * vid.get( CV_CAP_PROP_FPS );

      //     FrameVec_t frames;
      //     const int chunkSize = 50;
      //     Mat img;

      CompositeCanvas canvas;

      while( compVid.read( canvas ) ) {
             int currentFrame = compVid.get( CV_CAP_PROP_POS_FRAMES );
        
        for( int i = 0; i < 2; ++i ) {

          //Mat gray;
          //cvtColor( canvas[i], gray, COLOR_BGR2GRAY );

          Detection *det = board->detectPattern( canvas[i] );

          det->drawCorners( *board, canvas[i] );

          db[i].save( currentFrame, *det );
          delete det;
        }

        if( opts.doDisplay ) {
          imshow( BuildDbWindowName, canvas );
          int ch = waitKey( wk );
          if( ch == ' ' ) {
            wk = (wk == 0) ? wait : 0;
          }
        }

      }
      //       cout << currentFrame << " ";

      //       if( !db.has( currentFrame ) || opts.doRewrite ) {
      //         frames.push_back( Frame( currentFrame, img.clone() ) );
      //       }

      //       if( opts.intervalFrames > 1 ) {
      //         int destFrame = currentFrame + opts.intervalFrames - 1;

      //         if( destFrame < vidLength ) 
      //           vid.set( CV_CAP_PROP_POS_FRAMES, currentFrame + opts.intervalFrames - 1 );
      //         else 
      //           break;
      //       }

      //       if( frames.size() >= chunkSize ) {
      //         cout << endl;
      //         processFrames( frames );
      //       }
      //     }

      //     cout << endl;
      //     processFrames( frames );


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


