
#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <iomanip>
#include <string>
#include <sstream>
#include <fstream>
#include <cctype>
#include <algorithm>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <opencv2/ximgproc.hpp>

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
#include "disparity_db.h"

#include <boost/filesystem.hpp>

using namespace cv;
using namespace std;

using namespace boost::filesystem;

//using AprilTags::TagDetection;


using namespace AplCam;

struct Options
{

  typedef enum { DENSE_STEREO, UNDISTORT, RECTIFY, SWAP, VERB_NONE = -1 } Verb;
  typedef enum { STEREO_BM, STEREO_SGBM } StereoAlgorithm;

  // n.b. the default window should be a non-round number so you don't get an ambiguous number of second transitions...
  Options()
  : verb( VERB_NONE ), stereoCalibration(), videoFile(), doDisplay( false )

  {;}


  Verb verb;
  string stereoCalibration, cameraCalibrations[2], videoFile, outputFile,
          disparityDb, pointCloudDb;
  float scale, fastForward;
  int seekTo, stopAfter;
  bool doDisplay, doFilterDisparity;
  StereoAlgorithm stereoAlgorithm;

  bool parseOpts( int argc, char **argv, stringstream &msg )
  {

    try {

      TCLAP::CmdLine cmd("stereo_processor", ' ', "0.1");

      TCLAP::SwitchArg doDisplayArg("X", "do-display", "display", cmd, false );

      TCLAP::ValueArg<std::string> stereoCalibrationArg( "s", "stereo-calibration", "Stereo calibration file", true, "", "YAML file", cmd );
      TCLAP::ValueArg<std::string> calLeftArg("","left-camera", "Calibration file basename", true, "", "name", cmd );
      TCLAP::ValueArg<std::string> calRightArg("","right-camera", "Calibration file basename", true, "", "name", cmd );
      TCLAP::ValueArg<std::string> outputFileArg("o", "output-file", "Output file", false, "", "file name", cmd );

      TCLAP::ValueArg<std::string> stereoAlgorithmArg("","stereo-algorithm", "Stereo algorithm", false, "sgbm", "sgbm or bm", cmd );
      TCLAP::ValueArg<std::string> disparityDbArg("", "disparity-db", "Disparity d/b", false, "", "name of database", cmd );
TCLAP::ValueArg<std::string> pointcloudDbArg("", "point-cloud-db", "Point cloud d/b", false, "", "name of database", cmd );

      TCLAP::ValueArg<float> scaleArg("S", "scale", "Scale displayed output", false, 1.0, "scale factor", cmd );
      TCLAP::ValueArg<float> ffArg("F", "fast-forward", "Accelerate playback", false, 1.0, "factor", cmd );

      TCLAP::ValueArg<int> seekToArg("", "seek-to", "Seek to a frame", false, 0, "seek to" ,cmd );
      TCLAP::ValueArg<int> stopAfterArg("", "stop-after", "Process a limited number of frames", false, -1, "frames", cmd);

      TCLAP::SwitchArg doFilterDisparityArg("", "filter-disparity", "Do filter the disparity map", cmd, false );

      TCLAP::UnlabeledValueArg< std::string > verbArg( "verb", "Verb", true, "", "verb", cmd );
      TCLAP::UnlabeledValueArg< std::string > videoFileArg( "video-file", "Video file", true, "", "file name", cmd );

      cmd.parse( argc, argv );

      doDisplay = doDisplayArg.getValue();
      doFilterDisparity = doFilterDisparityArg.getValue();

      outputFile = path(outputFileArg.getValue()).replace_extension( VideoExtension ).string();
      videoFile = videoFileArg.getValue();
      cameraCalibrations[0] = calLeftArg.getValue();
      cameraCalibrations[1] = calRightArg.getValue();
      scale = scaleArg.getValue();
      fastForward = ffArg.getValue();

      seekTo = seekToArg.getValue();
      stopAfter = stopAfterArg.getValue();

      disparityDb = disparityDbArg.getValue();
      pointCloudDb = pointcloudDbArg.getValue();

      stereoCalibration = stereoCalibrationArg.getValue();

      string alg = stereoAlgorithmArg.getValue();
      std::transform(alg.begin(), alg.end(), alg.begin(), static_cast < int(*)(int) >(toupper) );

      if( alg == "BM")
      stereoAlgorithm = STEREO_BM;
      else if( alg == "SGBM")
      stereoAlgorithm = STEREO_SGBM;
      else {
        LOG(ERROR) << "Didn't understand stereo algorithm \"" << alg << "\"";
        return false;
      }

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
      return doRectify() ? 0 : -1;
      break;
      case Options::UNDISTORT:
      return doUndistort() ? 0 : -1;
      break;
      case Options::DENSE_STEREO:
      return doDenseStereo() ? 0 : -1;
      break;
      case Options::SWAP:
      return doSwap() ? 0 : -1;
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

          if( opts.scale  != 1.0 )
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

          if( opts.scale  != 1.0 )
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
      bool cvtToGrey = true;
      const int minDisparity = 0;

      // Note this is maxDisparity - minDisparity
      // Must be divisble by 16.
      const int numDisparities = 960; //(video.frameSize().width / 8 + 15) & -16;

      const int SADWindowSize  = 5;

      const int disp12MaxDiff = -1;

      const int speckleWindowSize = 0;
      const int speckleRange = 8;

      const int uniquenessRatio = 0;

      // Default paramters for WLS filtering (if used)
      const float wlsFilterLambda = 8000.0;
      const float wlsFilterSigmaColor = 2.0;

      //int SADWindowSize = 0;

      //Ptr<StereoBM> bm( StereoBM::create( numberOfDisparities ) );
      //     StereoSGBM sgbm;

      PointCloudDb pcDb;
      if( opts.pointCloudDb.length() > 0 ) {
        LOG(INFO) << "Saving point cloud to db at : " << opts.pointCloudDb;
        if( !pcDb.open( opts.pointCloudDb, true ) ) {
          LOG(ERROR) << "Error opening point cloud database: " << pcDb.error().name() << " : " << pcDb.error().message();
          return false;
        }

        pcDb.setFps( video.fps () );
      }

      DisparityMatDb disparityDb;
      if( opts.disparityDb.length() > 0 ) {
        LOG(INFO) << "Saving disparitys to db at : " << opts.disparityDb;
        if( !disparityDb.open( opts.disparityDb, true ) ) {
          LOG(ERROR) << "Error opening disparity database: " << disparityDb.error().name() << " : " << disparityDb.error().message();
          return false;
        }

        disparityDb.setFps( video.fps () );
      }


      Ptr<StereoMatcher> bm;

      if( opts.stereoAlgorithm == Options::STEREO_BM ) {
        const int preFilterSize = 5;
        const int preFilterCap = 61;

        LOG(INFO) << "Using StereoBM algorithm";
        Ptr<StereoBM> sbm( StereoBM::create(numDisparities, SADWindowSize ) );
        sbm->setUniquenessRatio( uniquenessRatio );
        sbm->setMinDisparity( minDisparity );
        sbm->setPreFilterCap( preFilterCap );
        sbm->setPreFilterSize( preFilterSize );

        sbm->setTextureThreshold(0);

        bm = sbm;
      } else if( opts.stereoAlgorithm == Options::STEREO_SGBM ) {
        const int p1 = 24 * SADWindowSize * SADWindowSize;
        const int p2 = 4 * p1;
        const int preFilterCap = 4;

        LOG(INFO) << "Using Semi-Global StereoBM algorithm";
        Ptr<StereoSGBM> sgbm( StereoSGBM::create( minDisparity, numDisparities, SADWindowSize, p1, p2 ) );
        sgbm->setPreFilterCap( preFilterCap );
        sgbm->setMode(StereoSGBM::MODE_SGBM_3WAY);

        cvtToGrey = false;

        bm = sgbm;
      } else {
        LOG(ERROR) << "Dense stereo algorithm is undefined.";
        return false;
      }

      // Arguments common to both algorithms
      bm->setSpeckleRange( speckleRange );
      bm->setSpeckleWindowSize( speckleWindowSize );
      bm->setDisp12MaxDiff( disp12MaxDiff );

      int wk = 1000 * 1.0/(video.fps() * opts.fastForward);
      if( wk < 1 ) wk = 1;
      //cout << "wk: " << wk << endl;
      int wait = wk;

      int count = 0;
      CompositeCanvas canvas;
      while( video.read( canvas ) ) {

        if( (opts.stopAfter > 0) && (count >= opts.stopAfter )) break;

        int type = cvtToGrey ? CV_8UC1 : CV_8UC3;
        CompositeCanvas scaled(  Size( canvas.size().width * opts.scale,
        canvas.size().height * opts.scale), type );
        for( int k = 0; k < 2; ++k )  {
          remap( canvas[k], canvas[k], map[k][0], map[k][1], INTER_LINEAR );

          // Apply scale before processing
          if( opts.scale != 1.0 ) {
            Mat resized;
            resize( canvas[k], resized, Size(), opts.scale, opts.scale, cv::INTER_LINEAR );

            if( cvtToGrey )
            cvtColor( resized, scaled[k], CV_BGR2GRAY );
            else
            resized.copyTo( scaled[k] ); // Inefficient...

          } else {

            if( cvtToGrey )
            cvtColor( canvas[k], scaled[k], CV_BGR2GRAY );
            else
            canvas[k].copyTo( scaled[k] ); // Inefficient...

          }

        }

        Mat disparity;
        if( opts.doFilterDisparity ) {
          LOG(INFO) << "Filtering disparity map with WLS filter";

          //Rect ROI( computeROI( scaled[0].size(), bm ) );

          // Hm, this actually changes parameters in the BM.  Is that right?
          //    matcher_left->setDisp12MaxDiff(1000000);
          //    matcher_left->setSpeckleWindowSize(0);
          //    if( StereoSGBM ) sgbm->setUniquenessRatio(0);
          Ptr<ximgproc::DisparityWLSFilter> wlsDisparityFilter( ximgproc::createDisparityWLSFilter( bm ) );
          Ptr<ximgproc::DisparityFilter>  disparityFilter( wlsDisparityFilter );
          wlsDisparityFilter->setLambda( wlsFilterLambda );
          wlsDisparityFilter->setSigmaColor( wlsFilterSigmaColor );

          Ptr< StereoMatcher > rightMatcher = ximgproc::createRightMatcher( bm );
          // These are the things being set in the right matcher
          //right_sgbm->setUniquenessRatio(0);
          // right_sgbm->setP1(sgbm->getP1());
          // right_sgbm->setP2(sgbm->getP2());
          // right_sgbm->setMode(sgbm->getMode());
          // right_sgbm->setPreFilterCap(sgbm->getPreFilterCap());
          // right_sgbm->setDisp12MaxDiff(1000000);
          // right_sgbm->setSpeckleWindowSize(0);

          Mat leftDisparity;
          int64 t = getTickCount();
          bm->compute( scaled[0], scaled[1], leftDisparity );
          t = getTickCount() - t;
          LOG(INFO) << video.frame() << " : Left-right dense stereo elapsed time (ms): " <<  t * 1000 / getTickFrequency();


          if( opts.doDisplay ) {
            Mat unfiltDispImage;
            disparityToImage( leftDisparity, unfiltDispImage, numDisparities );
            imshow( UnfilteredLeftWindowName, unfiltDispImage );
          }

          Mat rightDisparity;
          t = getTickCount();
          rightMatcher->compute( scaled[1], scaled[0], rightDisparity );
          t = getTickCount() - t;
          LOG(INFO) << video.frame() << " : Right-left dense stereo elapsed time (ms): " <<  t * 1000 / getTickFrequency() << endl;

          if( opts.doDisplay ) {
            Mat disparityImage;
            disparityToImage( rightDisparity, disparityImage, numDisparities );
            imshow( UnfilteredRightWindowName, disparityImage );
          }

          Mat filteredDisparity;
          t = (double)getTickCount();
          disparityFilter->filter(leftDisparity, scaled[0], filteredDisparity, rightDisparity, Rect(), scaled[1] );
          t = (getTickCount() - t);

          LOG(INFO) << video.frame() << " : Filtering elapsed time (ms): " <<  t * 1000 / getTickFrequency() << endl;

          disparity = filteredDisparity;

          Mat confMap( wlsDisparityFilter->getConfidenceMap() );
          if( opts.doDisplay ) {
            Mat scaledConfMap = confMap / 255.0;
            imshow( ConfidenceMapWindowName, scaledConfMap );
          }

        } else {
          int64 t = getTickCount();
          bm->compute( scaled[0], scaled[1], disparity );
          t = getTickCount() - t;
          LOG(INFO) << video.frame() << " : Dense stereo elapsed time (ms): " <<  t * 1000 / getTickFrequency();
        }


        Mat colorDisparity;
        disparityToImage( disparity, colorDisparity, numDisparities );

        if( writer.isOpened() ) {
          if( count % 100 == 0 ) { LOG(INFO) << count; }

          CompositeCanvas c( canvas[0], colorDisparity );

          writer << c;
        }

        if( disparityDb.isOpen() ) disparityDb.save( count, disparity );

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

    // Lifted from ximgproc/samples/disparity_filtering.cpp
    Rect computeROI(Size2i src_sz, Ptr<StereoMatcher> matcher_instance)
    {
      int min_disparity = matcher_instance->getMinDisparity();
      int num_disparities = matcher_instance->getNumDisparities();
      int block_size = matcher_instance->getBlockSize();

      int bs2 = block_size/2;
      int minD = min_disparity, maxD = min_disparity + num_disparities - 1;

      int xmin = maxD + bs2;
      int xmax = src_sz.width + minD - bs2;
      int ymin = bs2;
      int ymax = src_sz.height - bs2;

      Rect r(xmin, ymin, xmax - xmin, ymax - ymin);
      return r;
    }

    void disparityToImage( const Mat &disparity, Mat &dispImage, int numDisparities )
    {
      Mat disp8;
      double offset = 0;
      double minVal, maxVal;

      minMaxLoc( disparity, &minVal, &maxVal );
      // LOG(INFO) << "min value " << minVal;
      // LOG(INFO) << "max value " << maxVal;

      // Try to detect if it's a L-R or R-L disparity image
      if( (maxVal + minVal)/2.0 < 0.0 ) offset = 255;

      disparity.convertTo(disp8, CV_8U, 255/(numDisparities*16.), offset);
      dispImage.create( disp8.size(), CV_8UC3 );
      cvtColor( disp8, dispImage, CV_GRAY2BGR );
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


    static const string RectifyWindowName,
    UnfilteredLeftWindowName,
    UnfilteredRightWindowName,
    DenseStereoWindowName,
    ConfidenceMapWindowName;

  };

  const string StereoProcessorMain::RectifyWindowName = "Rectify",
  StereoProcessorMain::DenseStereoWindowName = "Dense Stereo",
  StereoProcessorMain::UnfilteredLeftWindowName = "Unfiltered Left Dense Stereo",
  StereoProcessorMain::UnfilteredRightWindowName = "Unfiltered Right Dense Stereo",
  StereoProcessorMain::ConfidenceMapWindowName = "ConfidenceMap";

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
