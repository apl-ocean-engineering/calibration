
#include <stdlib.h>
#include <getopt.h>
#include <time.h>

#include <iostream>
#include <iomanip>

#include <mutex>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <kchashdb.h>

#include <glog/logging.h>

#include <tclap/CmdLine.h>

#include "board.h"
#include "detection.h"
#include "detection_db.h"

#include "camera_factory.h"
using namespace AplCam;
using namespace Distortion;

using namespace std;
using namespace cv;
using kyotocabinet::HashDB;



struct SimulateDbOpts {
  public:
    SimulateDbOpts()
      : waitKey(33 ),
      duration(10),
      dataDir("data"),
      boardName(), 
      cameraName(),
      outputFile(),
      imageSize(),
      doDisplay( false )
  {;}

    int waitKey;
    unsigned int duration;
    string dataDir;
    string boardName, cameraName;
    string outputFile;
    Size imageSize;
    bool doDisplay;

    const string boardPath( void )
    { return dataDir + "/boards/" + boardName + ".yml"; }

    const string cameraPath( void )
    { return dataDir + "/cameras/" + cameraName + ".yml"; }

    const string tmpPath( const string &file )
    { return dataDir + "/tmp/" + file; }

    //== Option parsing and help ==

    bool parseArgs( int argc, char **argv, stringstream &msg ) 
    {

      try{
        TCLAP::CmdLine cmd("Detection simulator", ' ', "0.1");

        TCLAP::ValueArg< string > dataDirArg("D", "data-dir", "Data directory", false, "../data", "dir", cmd );
        TCLAP::ValueArg< string > boardNameArg("b", "board-name", "Board name", false, "april_poster_2in", "board name", cmd );
        TCLAP::ValueArg< string > cameraNameArg("c", "camera-name", "Camera name", false, "haptic4_sim", "camera name", cmd );
        TCLAP::ValueArg< string > imageSizeArg("i", "image-size", "Image size", false, "1920x1080", "HH x WW", cmd );

        TCLAP::SwitchArg displayArg("X", "display", "Show progress in window", cmd, false );
        TCLAP::ValueArg< int > waitKeyArg("w", "wait-key", "Arg for waitkey()", false, 33, "ms", cmd );

        TCLAP::ValueArg< unsigned int > durationArg("d", "duration", "Number of frames to generate", false, 100, "frames", cmd );

        TCLAP::UnlabeledValueArg< string > outputArg("output_file", "DB file to save results", true, "simulated.kch", "file name", cmd );

        cmd.parse( argc, argv );

        dataDir = dataDirArg.getValue();
        doDisplay = displayArg.getValue();
        waitKey = waitKeyArg.getValue();

        cameraName = cameraNameArg.getValue();
        boardName = boardNameArg.getValue();

        outputFile = outputArg.getValue();
        duration = durationArg.getValue();

        int width = 0, height=0;
        if( sscanf( imageSizeArg.getValue().c_str(), "%dx%d", &width, &height ) != 2 ) {
          throw TCLAP::ArgException("Couldn't parse size", "image size" );
        }

        imageSize = Size( width, height );

      } 
      catch (TCLAP::ArgException &e)  // catch any exceptions
      { 
        msg << "error: " << e.error() << " for arg " << e.argId() << std::endl;
        return false;
      }


      return validate( msg );
    }


    bool validate( stringstream &msg )
    {
      return true;
    }

};




class SimulateDbMain 
{
  public:
    SimulateDbMain( SimulateDbOpts &options )
      : opts( options )
    {;}

    ~SimulateDbMain( void )
    {
    }

    int run( void ) {
      if( opts.doDisplay ) namedWindow( WindowName );

      Board *board = Board::load( opts.boardPath(), opts.boardName );
      if( !board ) {
        LOG(ERROR) << "Couldn't open board from " << opts.boardPath() << endl;
        return -1;
      }

      DistortionModel *dist = CameraFactory::LoadDistortionModel( opts.cameraPath() );
      if( dist == NULL ) {
        LOG(ERROR) << "Couldn't load camera from " << opts.cameraPath() << endl;
        return -1;
      }

      if( ! db.open( opts.outputFile, true ) ) {
        LOG(ERROR) << "Error opening database file: " << db.error().name() << endl;
        return -1;
      }

      ObjectPointsVec corners = board->corners();
      vector< int > ids = board->ids();

      for( unsigned int i = 0; i < opts.duration; ++i ) {
        LOG(INFO) << "Frame " << i;

        Detection det;

        det.rot = pose.rvec;
        det.trans = pose.tvec;

        ImagePointsVec imgPts;
        dist->projectPoints( corners, pose.rvec, pose.tvec, imgPts );

        Size imgSize( opts.imageSize );

        // Selectively drop a few
        for( size_t i = 0; i < corners.size(); ++i ) {
          ImagePoint &pt( imgPts[i] );
          if( pt[0] < 0 || pt[0] > imgSize.width ||
              pt[1] < 0 || pt[1] > imgSize.height ) continue;
        //  if( rand() > (RAND_MAX/2) )
            det.add( corners[i], imgPts[i], ids[i] );
        }

        if( opts.doDisplay ) {
          Mat img = Mat::zeros( opts.imageSize, CV_8UC3 );

          for( ImagePointsVec::const_iterator itr = det.points.begin(); itr != det.points.end(); ++itr ) 
            circle( img, Point((*itr)[0], (*itr)[1]), 3, Scalar(0,0,255), 1 );

          imshow( WindowName, img );
          waitKey( opts.waitKey );
        }

        db.save( i, det );

        pose.update();
      }

      //      double vidLength = vid.get( CV_CAP_PROP_FRAME_COUNT );
      //
      //      if( opts.intervalSeconds > 0 ) 
      //        opts.intervalFrames = opts.intervalSeconds * vid.get( CV_CAP_PROP_FPS );
      //
      //      FrameVec_t frames;
      //      const int chunkSize = 50;
      //      Mat img;
      //
      //      while( vid.read( img ) ) {
      //        int currentFrame = vid.get( CV_CAP_PROP_POS_FRAMES );
      //        cout << currentFrame << " ";
      //
      //        if( !db.has( currentFrame ) || opts.doRewrite ) {
      //          frames.push_back( Frame( currentFrame, img.clone() ) );
      //        }
      //
      //        if( opts.intervalFrames > 1 ) {
      //          int destFrame = currentFrame + opts.intervalFrames - 1;
      //
      //          if( destFrame < vidLength ) 
      //            vid.set( CV_CAP_PROP_POS_FRAMES, currentFrame + opts.intervalFrames - 1 );
      //          else 
      //            break;
      //        }
      //
      //        if( frames.size() >= chunkSize ) {
      //          cout << endl;
      //          processFrames( frames );
      //        }
      //      }
      //
      //      cout << endl;
      //      processFrames( frames );


      return 0;
    }


    struct BoardPose {
      BoardPose( void )
        : tvec( 0, 0, 1000 ), rvec( 0, 0, 0 )
      {;}

      void update( void )
      {
        ;
      }

      Vec3d tvec, rvec;
    };


    const string WindowName = "SimulateDb";

  private:
    SimulateDbOpts opts;

    DetectionDb db;
    BoardPose pose;

    ofstream _benchmark;
};



int main( int argc, char **argv ) 
{
  FLAGS_logtostderr = 1;
  google::InitGoogleLogging( argv[0] );

  SimulateDbOpts opts;
  stringstream msg;
  if( !opts.parseArgs( argc, argv, msg ) ) {
    cout << msg.str() << endl;
    exit(-1);
  }

  SimulateDbMain main( opts );

  exit( main.run() );

}


