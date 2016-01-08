// #include "opencv2/core/core.hpp"
// #include "opencv2/imgproc/imgproc.hpp"
// #include "opencv2/calib3d/calib3d.hpp"
// #include "opencv2/highgui/highgui.hpp"

// #include <cctype>

#include <stdio.h>
// #include <string.h>
// #include <time.h>
// #include <getopt.h>
//
// #include <iostream>
//
// #include "my_undistort.h"
//
// #include "distortion/distortion_model.h"
// #include "distortion/angular_polynomial.h"
// #include "distortion/radial_polynomial.h"
// using namespace Distortion;
//
// #include "file_utils.h"
// #include "board.h"
// #include "detection/detection.h"
// #include "image.h"
// #include "calibration_serializer.h"
//
// #include "calibration_opts.h"

// using namespace cv;
// using namespace AplCam;

#include "cal_impl.h"


//class CalibrationOpts : public AplCam::CalibrationOptsCommon {
//
//  public:
//
//
//    CalibrationOpts()
//      : CalibrationOptsCommon(),
//      inFiles(),
//      ignoreCache( false ), retryUnregistered( false )
//  {;}
///
//    //== Option parsing and help ==
//    void help()
//    {
//      printf( "This is a camera calibration sample.\n"
//          "Usage: calibration\n"
//          "     -d <data directory>      # Specify top-level directory for board/camera/cache files.\n"
//          "     --board,-b <board_name>    # Name of calibration pattern\n"
//          "     --camera, -c <camera_name> # Name of camera\n"
//          "     --ignore-cache, -i       # Ignore and overwrite files in cache\n"
//          "     --retry-unregistered, -r   # Re-try to find the chessboard if the cache file is empty\n"
//          "     --calibration-model, -m   # Set the distortion model to: angular, radial, radial8\n"
//          "     --fix-skew, -k            # Fix skew (alpha) to 0\n"
//          //     "     [-d <delay>]             # a minimum delay in ms between subsequent attempts to capture a next view\n"
//          //     "                              # (used only for video capturing)\n"
//          //     "     [-o <out_camera_params>] # the output filename for intrinsic [and extrinsic] parameters\n"
//          //     "     [-op]                    # write detected feature points\n"
//          //     "     [-oe]                    # write extrinsic parameters\n"
//          //     "     [-zt]                    # assume zero tangential distortion\n"
//          //     "     [-a <aspectRatio>]      # fix aspect ratio (fx/fy)\n"
//          //     "     [-p]                     # fix the principal point at the center\n"
//          //     "     [-v]                     # flip the captured images around the horizontal axis\n"
//          //     "     [-V]                     # use a video file, and not an image list, uses\n"
//          //     "                              # [input_data] string for the video file name\n"
//          //     "     [-su]                    # show undistorted images after calibration\n"
//          "     [input_data]             # list of files to use\n"
//          "\n" );
//      //printf("\n%s",usage);
//      //printf( "\n%s", liveCaptureHelp );
//    }
//
//
//    void parseOpts( int argc, char **argv )
//    {
//      static struct option long_options[] = {
//        { "data-directory", true, NULL, 'D' },
//        { "board", true, NULL, 'b' },
//        { "camera", true, NULL, 'c' },
//        { "calibation-model", true, NULL, 'm' },
//        { "ignore-cache", false, NULL, 'i' },
//        { "fix-skew", false, NULL, 'k'},
//        { "retry-unregistered", false, NULL, 'r' },
//        { "calibration-file", required_argument, NULL, 'z' },
//        { "help", false, NULL, '?' },
//        { 0, 0, 0, 0 }
//      };
//
//
//      if( argc < 2 )
//      {
//        help();
//        exit(1);
//      }
//
//      int indexPtr;
//      int optVal;
//      string c;
//      while( (optVal = getopt_long( argc, argv, "irb:c:D:km:z:?", long_options, &indexPtr )) != -1 ) {
//        switch( optVal ) {
//          case 'z':
//            calibrationFile = optarg;
//            break;
//          case 'D':
//            dataDir = optarg;
//            break;
//          case 'b':
//            boardName = optarg;
//            break;
//          case 'c':
//            cameraName = optarg;
//            break;
//          case 'i':
//            ignoreCache = true;
//            break;
//          case 'k':
//            //calibFlags |= PinholeCamera::CALIB_FIX_SKEW;
//            cout << "Skew is always fixed." << endl;
//            break;
//          case 'm':
//            calibType = DistortionModel::ParseCalibrationType( optarg );
//            break;
//          case 'r':
//            retryUnregistered = true;
//            break;
//          case '?':
//            help();
//            break;
//          default:
//            exit(-1);
//
//        }
//      }
//
//      if( optind == argc )
//      {
//        cout << "No input files specified." << endl;
//        exit(-1);
//      }
//
//      for( int i = optind; i < argc; ++i ) {
//        string infile( argv[i] );
//
//        if( !file_exists( infile ) ) {
//          cout << "Couldn't open input file \"" << infile << "\"" << endl;
//          exit(-1);
//        }
//
//        inFiles.push_back( infile );
//      }
//
//      string msg;
//      if( !validate( msg ) ) {
//        cout << "Error: " <<  msg << endl;
//        exit(-1);
//      }
//    }
//
//
//
//};




int main( int argc, char** argv )
{
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;

  CalOpts opts;

  opts.parseOpts( argc, argv );

  Cal cal( opts );
  exit( cal.run() );

  //
  // Size imageSize;
  // //float aspectRatio = 1.f;
  // //bool writeExtrinsics = false, writePoints = false;
  //
  // Distortion::ImagePointsVecVec imagePoints;
  // Distortion::ObjectPointsVecVec objectPoints;
  //
  // if( opts.inFiles.size() < 1 ) {
  //   cout << "No input files specified on command line." << endl;
  //   exit(-1);
  // }
  //
  // vector<Image> imagesUsed;
  //
  // if( opts.ignoreCache ) cout << "Ignoring cached data." << endl;
  // for( size_t i = 0; i < opts.inFiles.size(); ++i ) {
  //   cout << "Processing " << i << " : " << opts.inFiles[i] << endl;
  //   Mat view, viewGray;
  //
  //   view = imread(opts.inFiles[i], 1);
  //
  //   imageSize = view.size();
  //
  //   //if(!view.data)
  //   //{
  //   //  if( imagePoints.size() > 0 )
  //   //    runAndSave(outputFilename, imagePoints, imageSize,
  //   //        boardSize, pattern, squareSize, aspectRatio,
  //   //        flags, cameraMatrix, distCoeffs,
  //   //        writeExtrinsics, writePoints);
  //   //  break;
  //   //}
  //
  //   Image img( opts.inFiles[i], view );
  //   Detection *detection = NULL;
  //
  //   // Check for cached data
  //   string detectionCacheFile = opts.imageCache( img );
  //   bool doRegister = true;
  //
  //   if( !opts.ignoreCache && (detection = Detection::loadCache( detectionCacheFile )) != NULL ) {
  //     doRegister = false;
  //     if( opts.retryUnregistered && detection && (detection->points.size() == 0) ) doRegister = true;
  //   }
  //
  //   if( doRegister == false ) {
  //     cout << "  ... loaded data from cache." << endl;
  //   } else {
  //
  //     cout << "  No cached data, searching for calibration pattern." << endl;
  //
  //     //if( flipVertical )
  //     //  flip( view, view, 0 );
  //
  //     cvtColor(view, viewGray, COLOR_BGR2GRAY);
  //
  //     detection = board->detectPattern( viewGray );
  //
  //     if( detection->found )
  //       cout << "  Found calibration pattern." << endl;
  //
  //     cout << "Writing to " << detectionCacheFile << endl;
  //     detection->writeCache( *board, detectionCacheFile );
  //   }
  //
  //   if( detection->points.size() > 3 ) {
  //     imagesUsed.push_back( img );
  //
  //     //Whoops.  Type conversion from vec<Point2f> to vec<Vec2f> that needs to be cleaned up later
  //     ImagePointsVec imgPts( detection->points.size() );
  //     std::copy( detection->points.begin(), detection->points.end(), imgPts.begin() );
  //
  //     ObjectPointsVec wldPts( detection->corners.size() );
  //     std::copy( detection->corners.begin(), detection->corners.end(), wldPts.begin() );
  //
  //     imagePoints.push_back( imgPts );
  //     objectPoints.push_back( wldPts );
  //
  //     detection->drawCorners(  *board, view );
  //   }
  //
  //   string outfile( opts.tmpPath( img.basename() ) );
  //   mkdir_p( outfile );
  //   imwrite(  outfile, view );
  //
  //   delete detection;
  // }
  //
  //
  // cout << "Using points from " << imagePoints.size() << "/" << opts.inFiles.size() << " images" << endl;
  //
  // if( imagePoints.size() < 3 ) {
  //   cerr << "Not enough images.  Stopping." << endl;
  //   exit(-1);
  // }
  //
  //
  // int flags =  opts.calibFlags();
  //
  // DistortionModel *distModel = DistortionModel::MakeDistortionModel( opts.calibType );
  //
  // if( !distModel ) {
  //   cerr << "Something went wrong making a distortion model." << endl;
  //   exit(-1);
  // }
  //
  // CalibrationResult result;
  // distModel->calibrate( objectPoints, imagePoints,
  //     imageSize, result, flags );
  //
  // //  ///*|CV_CALIB_FIX_K3*/|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
  // printf("RMS error reported by calibrateCamera: %g\n", result.rms);
  //   //cout << "Residual reported by calibrateCamera: " << result.residual << endl;
  //
  // //  bool ok = checkRange(cameraMatrix) && checkRange(distCoeffs);
  //
  // bool ok = true;
  //
  // vector<float> reprojErrs;
  //
  // if( ok ) {
  //   CalibrationSerializer ser;
  //   ser.setCamera( distModel )
  //      .setResult( &result )
  //      .setBoard( board );
  //
  //   if( !ser.writeFile( opts.calibrationFile ) ) {
  //     cerr << "Error writing calibration to " << opts.calibrationFile << endl;
  //   } else {
  //     cout << "Wrote calibration to " << opts.calibrationFile<< endl;
  //   }
  //
  //  }
  //
  // //
  // //
  // //  // Redraw each image with rectified points
  // double alpha = 1;   // As a reminder, alpha = 0 means all pixels in undistorted image are correct
  // //                      //                alpha = 1 means all source image pixels are included
  // //                      //
  //
  // Rect validROI;
  // Mat optimalCameraMatrix = distModel->getOptimalNewCameraMatrix( imageSize, alpha, Size(), validROI );
  //
  // //Mat optimalCameraMatrix = distModel->mat();
  //
  // //  cout << "Distortion coefficients: " << endl << distCoeffs << endl;
  // cout << "Calculated camera matrix: " << endl << distModel->mat() << endl;
  // cout << "Optimal camera matrix: " << endl << optimalCameraMatrix << endl;
  //
  //
  // Mat map1, map2;
  // distModel->initUndistortRectifyMap( Mat::eye(3,3,CV_64F), optimalCameraMatrix, imageSize, CV_16SC2, map1, map2);
  //
  // //  //fisheye::initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat::eye(3,3,CV_64F), cameraMatrix,
  // //  //    imageSize, CV_16SC2, map1, map2);
  //
  // for( size_t i = 0; i < imagesUsed.size(); ++i ) {
  //
  //   string outfile;
  //   if( objectPoints[i].size() > 0 ) {
  //     Distortion::ImagePointsVec reprojImgPoints;
  //     Mat out;
  //     //fisheye::projectPoints(Mat(objectPoints[i]), imagePoints2, rvecs[i], tvecs[i],
  //     //   cameraMatrix, distCoeffs);
  //     distModel->projectPoints(objectPoints[i], result.rvecs[i], result.tvecs[i], reprojImgPoints );
  //
  //     imagesUsed[i].img().copyTo( out );
  //     for( size_t j = 0; j < imagePoints[i].size(); ++j ) {
  //       circle( out, Point2f(imagePoints[i][j]), 5, Scalar(0,0,255), 1 );
  //
  //       circle( out, Point2f(reprojImgPoints[j]), 5, Scalar(0,255,0), 1 );
  //     }
  //
  //
  //     outfile  = opts.tmpPath( String("reprojection/") +  imagesUsed[i].basename() );
  //     mkdir_p( outfile );
  //     imwrite(  outfile, out );
  //   }
  //
  //
  //   Mat rview;
  //   remap( imagesUsed[i].img(), rview, map1, map2, INTER_LINEAR);
  //
  //   const int N = 9;
  //   int x, y, k;
  //   ImagePointsVec pts, undPts;
  //
  //   for( y = k = 0; y < N; y++ )
  //     for( x = 0; x < N; x++ )
  //       pts.push_back( ImagePoint((float)x*imageSize.width/(N-1),
  //             (float)y*imageSize.height/(N-1)) );
  //
  //   distModel->undistortPoints(pts, undPts, Mat(), optimalCameraMatrix );
  //
  //   for( y = k = 0; y < N; y++ )
  //     for( x = 0; x < N; x++ ) {
  //       //        cout << pts[k] <<  "  " << undPts[k] << endl;
  //       Point2d thisPt( undPts[k++] );
  //       if( thisPt.x >=0 && thisPt.x <= rview.size().width &&
  //           thisPt.y >=0 && thisPt.y <= rview.size().height )
  //         circle( rview, thisPt, 5, Scalar(0,255,255), 2 );
  //     }
  //
  //
  //   outfile = opts.tmpPath( String("undistorted/") +  imagesUsed[i].basename() );
  //   mkdir_p( outfile );
  //   imwrite(  outfile, rview );
  //
  // }
  //
  // delete distModel;
  //
  // //    if( inputFilename )
  // //    {
  // //        if( !videofile && readStringList(inputFilename, imageList) )
  // //            mode = CAPTURING;
  // //        else
  // //            capture.open(inputFilename);
  // //    }
  // //    else
  // //        capture.open(cameraId);
  // //
  // //    if( !capture.isOpened() && imageList.empty() )
  // //        return fprintf( stderr, "Could not initialize video (%d) capture\n",cameraId ), -2;
  // //
  // //    if( !imageList.empty() )
  // //        nframes = (int)imageList.size();
  // //
  // //    if( capture.isOpened() )
  // //        printf( "%s", liveCaptureHelp );
  // //
  // //    namedWindow( "Image View", 1 );
  // //
  // //    for(i = 0;;i++)
  // //    {
  // //
  // //    if( !capture.isOpened() && showUndistorted )
  // //    {
  // //        Mat view, rview, map1, map2;
  // //        initUndistortRectifyMap(cameraMatrix, distCoeffs, Mat(),
  // //                                getOptimalNewCameraMatrix(cameraMatrix, distCoeffs, imageSize, 1, imageSize, 0),
  // //                                imageSize, CV_16SC2, map1, map2);
  // //
  // //        for( i = 0; i < (int)imageList.size(); i++ )
  // //        {
  // //            view = imread(imageList[i], 1);
  // //            if(!view.data)
  // //                continue;
  // //            //undistort( view, rview, cameraMatrix, distCoeffs, cameraMatrix );
  // //            remap(view, rview, map1, map2, INTER_LINEAR);
  // //            imshow("Image View", rview);
  // //            int c = 0xff & waitKey();
  // //            if( (c & 255) == 27 || c == 'q' || c == 'Q' )
  // //                break;
  // //        }
  // //    }
  // //
  //
  //
}
