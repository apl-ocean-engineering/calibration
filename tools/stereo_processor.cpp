
#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <fstream>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include <getopt.h>

#include <tclap/CmdLine.h>
#include <glog/logging.h>

#include "file_utils.h"
#include "composite_canvas.h"
#include "stereo_calibration.h"

#include "distortion/distortion_model.h"
#include "distortion/distortion_stereo.h"
#include "distortion/camera_factory.h"
using namespace Distortion;

#include "video_prefs.h"

#include <boost/filesystem.hpp>

using namespace cv;
using namespace std;

using namespace boost::filesystem;

//using AprilTags::TagDetection;


using namespace AplCam;

struct Options
{

  typedef enum { DENSE_STEREO, UNDISTORT, RECTIFY, SWAP, VERB_NONE = -1 } Verb;

  // n.b. the default window should be a non-round number so you don't get an ambiguous number of second transitions...
  Options()
      : verb( VERB_NONE ), stereoCalibration(), videoFile(), doDisplay( false )

  {;}


  Verb verb;
  string stereoCalibration, cameraCalibrations[2], videoFile, outputFile;
  float scale, fastForward;
  int seekTo;
  bool doDisplay;

  bool parseOpts( int argc, char **argv, stringstream &msg )
  {

    try {

      TCLAP::CmdLine cmd("stereo_processor", ' ', "0.1");

      TCLAP::SwitchArg doDisplayArg("X", "do-display", "display", cmd, false );

      TCLAP::ValueArg<std::string> stereoCalibrationArg( "s", "stereo-calibration", "Stereo calibration file", true, "", "YAML file", cmd );
      TCLAP::ValueArg<std::string> calLeftArg("0","camera-left", "Calibration file basename", true, "", "name", cmd );
      TCLAP::ValueArg<std::string> calRightArg("1","camera-right", "Calibration file basename", true, "", "name", cmd );
      TCLAP::ValueArg<std::string> outputFileArg("o", "output-file", "Output file", false, "", "file name", cmd );

      TCLAP::ValueArg<float> scaleArg("S", "scale", "Scale displayed output", false, -1.0, "scale factor", cmd );
      TCLAP::ValueArg<float> ffArg("F", "fast-forward", "Accelerate playback", false, 1.0, "factor", cmd );

      TCLAP::ValueArg<int> seekToArg("", "seek-to", "Seek to a frame", false, 0, "seek to" ,cmd );

      //TCLAP::SwitchArg doAnnotateArg("N", "do-annotate", "Do annotate entries into a video", cmd, false );

      TCLAP::UnlabeledValueArg< std::string > verbArg( "verb", "Verb", true, "", "verb", cmd );
      TCLAP::UnlabeledValueArg< std::string > videoFileArg( "video-file", "Video file", true, "", "file name", cmd );

      cmd.parse( argc, argv );

      doDisplay = doDisplayArg.getValue();

      outputFile = path(outputFileArg.getValue()).replace_extension( VideoExtension ).string();
      videoFile = videoFileArg.getValue();
      cameraCalibrations[0] = calLeftArg.getValue();
      cameraCalibrations[1] = calRightArg.getValue();
      scale = scaleArg.getValue();
      fastForward = ffArg.getValue();

      seekTo = seekToArg.getValue();

      stereoCalibration = stereoCalibrationArg.getValue();

      string v = verbArg.getValue();
      if( v == "rectify" ) {
        verb = RECTIFY;
      } else if( v == "undistort" ) {
        verb = UNDISTORT;
      } else if( v == "dense" || v == "dense_stereo" ) {
        verb = DENSE_STEREO;
      } else if( v == "swap" ) {
        verb = SWAP;
      } else {
        LOG(ERROR) << "Didn't understand verb \"" << v << "\"";
        return false;
      }


    } catch( TCLAP::ArgException &e ) {
      LOG( ERROR ) << "error: " << e.error() << " for arg " << e.argId();
    }

    return validate( msg );
  }

  bool validate( stringstream &msg )
  {
    for( int i = 0; i < 2; ++i )  {
      if( !file_exists( cameraCalibrations[i] ) ) {
        msg << "Can't find camera file: " <<   cameraCalibrations[i];
        return false;
      }
    }

    if( !file_exists( stereoCalibration ) ) {
      msg << "Can't find stereo calibration: " << stereoCalibration;
      return false;
    }

    return true;
  }

  bool outputFileGiven( void ) { return outputFile.length() > 0; }

};


class StereoProcessorMain {
 public:
  StereoProcessorMain( Options &opt )
      : opts( opt ), video( opt.videoFile )
  {;}

  int go( void )
  {
    if( ! video.isOpened() ) {
      LOG(ERROR) << "Could not open video file: " << opts.videoFile << endl;
      return -1;
    }

    if( opts.seekTo > 0 )
    {
      LOG(INFO) << "Seeking to " << opts.seekTo;
      if( video.seek( opts.seekTo ) == false ) {
        LOG(ERROR) << "Video would not seek to " << opts.seekTo;
      }
    }

    if( loadCalibrationFiles() == false ) return -1;

    if( opts.outputFileGiven() ) {
      float fps = video.fps();
      if( isnan( fps ) ) fps = 30.0;
      LOG(INFO) << "Fps: " << fps;

      Size outputSize( video.fullSize() );
      //switch( opts.verb ) {
      //  case Options::RECTIFY:
      //    outputSize = video.fullSize();
      //    break;
      //  case Options::DENSE_STEREO:
      //    outputSize = video.frameSize();
      //    break;
      //}

      writer.open( opts.outputFile, VideoCodec, fps, outputSize );

      if( !writer.isOpened() ) {
        LOG(ERROR) << "Error creating writer for " << opts.outputFile;
        return -1;
      }
    }

    switch( opts.verb ) {
      case Options::RECTIFY:
        return doRectify();
        break;
      case Options::UNDISTORT:
        return doUndistort();
        break;
      case Options::DENSE_STEREO:
        return doDenseStereo();
        break;
      case Options::SWAP:
        return doSwap();
        break;
      case Options::VERB_NONE:
      default:
        LOG(ERROR) << "No verb selected, oh well." << endl;
        return 0;
    }

    return 0;
  }

 private:
  Options &opts;
  CompositeVideo video;

  StereoCalibration sCal;
  StereoRectification sRect;
  Mat map[2][2];

  DistortionModel *cameras[2];

  VideoWriter writer;


  void calculateRectification( void )
  {
    Mat R[2], P[2], disparity;
    Rect validROI[2];
    float alpha = -1;

    Distortion::stereoRectify( *cameras[0], *cameras[1], video.frameSize(), sCal.R, sCal.t,
                              R[0], R[1], P[0], P[1],  disparity, CALIB_ZERO_DISPARITY,
                              alpha, video.frameSize(), validROI[0], validROI[1] );

    sRect.R[0] = R[0];
    sRect.R[1] = R[1];
    sRect.P[0] = P[0];
    sRect.P[1] = P[1];

    Size frameSize( video.frameSize() );
    for( int k = 0; k < 2; ++k ) {
      cameras[k]->initUndistortRectifyMap( sRect.R[k], sRect.P[k],
                                          frameSize, CV_32FC1, map[k][0], map[k][1] );
    }
  }

  int doSwap( void )
  {

    int count = 0;
    CompositeCanvas in;
    while( video.read( in ) ) {

      CompositeCanvas out( in.size(), CV_8UC3  );
      in[0].copyTo(out[1]);
      in[1].copyTo(out[0]);

      if( writer.isOpened() ) {
        if( count % 100 == 0 ) { LOG(INFO) << count; }
        writer << out;
      }

      ++count;
    }

    return 0;
  }



  int doRectify( void )
  {
    LOG(INFO) << "Fps: " << video.fps();
    int wk = 1000 * 1.0/(video.fps() * opts.fastForward);
    int wait = wk;

    int count = 0;
    CompositeCanvas canvas;
    while( video.read( canvas ) ) {
      for( int k = 0; k < 2; ++k )
        remap( canvas[k], canvas[k], map[k][0], map[k][1], INTER_LINEAR );

      if( writer.isOpened() ) {
        if( count % 100 == 0 ) { LOG(INFO) << count; }
        writer << canvas;
      }

      if( opts.doDisplay ) {

        if( opts.scale  > 0.0 )
          imshow( RectifyWindowName, canvas.scaled( opts.scale ) );
        else
          imshow( RectifyWindowName, canvas );

        int ch;
        ch = waitKey( wait );
        if( ch == 'q' )
          break;
        else if(ch==' ') {
          wait = (wait == 0) ? wk : 0;
        }
      }

      ++count;
    }

    return 0;
  }

  int doUndistort( void )
  {
    LOG(INFO) << "Fps: " << video.fps();
    int wk = 1000 * 1.0/(video.fps() * opts.fastForward);
    int wait = wk;

    int count = 0;
    CompositeCanvas canvas;
    while( video.read( canvas ) ) {

      undistortCanvas( canvas );

      if( writer.isOpened() ) {
        if( count % 100 == 0 ) { LOG(INFO) << count; }
        writer << canvas;
      }

      if( opts.doDisplay ) {

        if( opts.scale  > 0.0 )
          imshow( RectifyWindowName, canvas.scaled( opts.scale ) );
        else
          imshow( RectifyWindowName, canvas );

        int ch;
        ch = waitKey( wait );
        if( ch == 'q' )
          break;
        else if(ch==' ') {
          wait = (wait == 0) ? wk : 0;
        }
      }

      ++count;
    }

    return 0;
  }




  bool doDenseStereo( void )
  {
    const int numDisparities = (video.frameSize().width / 8 + 15) & -16;
    const int minDisparity = 0;
    const int blockSize = 13;

    const int SADWindowSize  = 9;
    const int p1 = 2 * SADWindowSize * SADWindowSize;
    const int p2 = 4 * p1;
    const int speckleWindowSize = 50;
    const int speckleRange = 32;
    const int uniquenessRatio = 15;

    //int SADWindowSize = 0;

    //Ptr<StereoBM> bm( StereoBM::create( numberOfDisparities ) );
    //     StereoSGBM sgbm;

    Ptr<StereoSGBM> bm( StereoSGBM::create( minDisparity, numDisparities, blockSize ) );

    //bm->setSpeckleRange( speckleRange );
    //bm->setSpeckleWindowSize( speckleWindowSize );
    //bm->setUniquenessRatio( uniquenessRatio );
    bm->setP1( p1 );
    bm->setP2( p2 );

    ///bm->setMinDisparity( 0 );
    ///bm->setSpeckleRange( 32 );
    ///bm->setSpeckleWindowSize( 100 );

    //     // bm.state->roi1 = roi1;
    //     // bm.state->roi2 = roi2;
    //     // bm.state->preFilterCap = 31;
    //     // bm.state->SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 9;
    //bm.state->minDisparity = 0;
    //bm.state->numberOfDisparities = numberOfDisparities;
    //bm.state->textureThreshold = 10;
    //bm.state->uniquenessRatio = 15;
    //bm.state->speckleWindowSize = 100;
    //bm.state->speckleRange = 32;
    //bm.state->disp12MaxDiff = 1;

    //     sgbm.preFilterCap = 63;
    //     sgbm.SADWindowSize = 3;

    //     //int cn = canvas.canvas.channels();

    //     sgbm.P1 = 8*sgbm.SADWindowSize*sgbm.SADWindowSize;
    //     sgbm.P2 = 32*sgbm.SADWindowSize*sgbm.SADWindowSize;
    //     sgbm.minDisparity = 0;
    //     sgbm.numberOfDisparities = numberOfDisparities;
    //     sgbm.uniquenessRatio = 10;
    //     //sgbm.speckleWindowSize = bm.state->speckleWindowSize;
    //     //sgbm.speckleRange = bm.state->speckleRange;
    //     sgbm.disp12MaxDiff = 1;
    //     sgbm.fullDP = false;


    int wk = 1000 * 1.0/(video.fps() * opts.fastForward);
    if( wk < 1 ) wk = 1;
    cout << "wk: " << wk << endl;
    int wait = wk;

    int count = 0;
    CompositeCanvas canvas;
    while( video.read( canvas ) ) {

      CompositeCanvas scaled(  Size( canvas.size().width * opts.scale,
                                     canvas.size().height * opts.scale), CV_8UC1 );
      for( int k = 0; k < 2; ++k )  {
        remap( canvas[k], canvas[k], map[k][0], map[k][1], INTER_LINEAR );

        // Apply scale before processing
        if( opts.scale > 0 ) {
          Mat resized;
          resize( canvas[k], resized, Size(), opts.scale, opts.scale, cv::INTER_LINEAR );
          cvtColor( resized, scaled[k], CV_BGR2GRAY );
        } else {
          cvtColor( canvas[k], scaled[k], CV_BGR2GRAY );
        }

      }

      Mat disparity;
      int64 t = getTickCount();
      bm->compute( scaled[0], scaled[1], disparity );
      t = getTickCount() - t;
      LOG(INFO) << video.frame() << " : Dense stereo elapsed time (ms): " <<  t * 1000 / getTickFrequency() << endl;

      Mat disp8;
      disparity.convertTo(disp8, CV_8U, 255/(numDisparities*16.));
      Mat colorDisparity( disp8.size(), CV_8UC3 );
      cvtColor( disp8, colorDisparity, CV_GRAY2BGR );

      if( writer.isOpened() ) {
        if( count % 100 == 0 ) { LOG(INFO) << count; }

        CompositeCanvas c( canvas[0], colorDisparity );

        writer << c;
      }

      if( opts.doDisplay ) {

          imshow( RectifyWindowName, scaled );
        imshow( DenseStereoWindowName, colorDisparity );
        int ch;
//int w = std::max( (int64)10, wait - t*1000 );
        ch = waitKey( wait );

        if( ch == 'q' )
          break;
        else if(ch==' ') {
          wait = (wait == 0) ? wk : 0;
        }
      }

      ++count;

    }


    //     Mat disparity;
    //     int64 t = getTickCount();
    //     //bm( scaled[0], scaled[1], disparity );
    //     sgbm( scaled[0], scaled[1], disparity );
    //     t = getTickCount() - t;
    //     cout << "Dense stereo elapsed time (ms): " <<  t * 1000 / getTickFrequency() << endl;

    //     Mat disp8;
    //     disparity.convertTo(disp8, CV_8U, 255/(numberOfDisparities*16.));

    //     imshow( "disparity", disp8 );

    return true;
  }

  bool undistortCanvas( CompositeCanvas &canvas )
  {
    for( int k = 0; k < 2; ++k )
      cameras[k]->undistortImage( canvas[k], canvas[k] );

    return true;
  }


  bool loadCalibrationFiles( void )
  {

    sCal.load( opts.stereoCalibration );

    for( int k =0; k < 2; ++k ) {
      cameras[k] = CameraFactory::LoadDistortionModel( opts.cameraCalibrations[k] );

      if( cameras[k] == NULL) {
        cerr << "Error loading calibration file " + opts.cameraCalibrations[k] << endl;
        return false;
      }
    }

    calculateRectification();

    return true;
  }


  static const string RectifyWindowName, DenseStereoWindowName;

};

const string StereoProcessorMain::RectifyWindowName = "Rectify",
      StereoProcessorMain::DenseStereoWindowName = "Dense Stereo";

int main( int argc, char **argv )
{
  google::InitGoogleLogging( argv[0] );
  FLAGS_logtostderr = 1;

  Options opts;
  stringstream msg;
  if( !opts.parseOpts( argc, argv, msg ) ) {
    cout << msg.str() << endl;
    exit(-1);
  }

  StereoProcessorMain main( opts );

  exit( main.go() );
}
