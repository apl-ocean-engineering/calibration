#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include <iostream>
#include <vector>
#include <algorithm>
#include <iterator>
#include <stdio.h>

#include <AprilTags/TagFamily.h>
#include <AprilTags/Tag16h5.h>
#include <AprilTags/TagDetector.h>

#include "april_tag_board_generator.h"
#include "april_tag_detection_set.h"

using namespace cv;
using namespace std;

static void help()
{
  printf( "\nThis code generates an artificial camera and artificial chessboard images,\n"
      "and then calibrates. It is basically test code for calibration that shows\n"
      "how to package calibration points and then calibrate the camera.\n"
      "Usage:\n"
      "./calibration_artificial\n\n");
}



const Size imgSize(1920,1080);
const Size brdSize(20,20);
const size_t brds_num =1;

template<class T> ostream& operator<<(ostream& out, const Mat_<T>& mat)
{
  for(int j = 0; j < mat.rows; ++j)
    for(int i = 0; i < mat.cols; ++i)
      out << mat(j, i) << " ";
  return out;
}


Mat generateBackground( void )
{
  cout << "Initializing background...";
  Mat background(imgSize, CV_8UC3);
  randu(background, Scalar::all(32), Scalar::all(255));
  GaussianBlur(background, background, Size(5, 5), 2);
  cout << "Done" << endl;

  return background;
}



int main( int argc, char **argv )
{
  const char *mainWindow = "Current image";

  help();

  // Initialize the RNG
  cv::theRNG().state = cv::getTickCount();


  Mat background( generateBackground() );

  AprilTags::TagCodes tagCodes = AprilTags::tagCodes16h5;
  AprilTags::TagDetector tagDetector( tagCodes );

  cout << "Initializing April board...";
  AprilTagBoard apb( tagCodes, brdSize );

  AprilTagBoardGenerator apbGen( apb );
  cout << " Done" << endl;

  // Initialize camera parameters
  Mat_<double> camMat(3, 3);
  camMat << 300., 0., background.cols/2., 0, 300., background.rows/2., 0., 0., 1.;

  Mat_<double> distCoeffs(1, 5);
  distCoeffs << 1.2, 0.2, 0., 0., 0.;
  //distCoeffs << 0.0, 0.0, 0., 0., 0.;

  cout << "Generating chessboards...";
  vector<SimulatedImage> boards;
  for(size_t i = 0; i < brds_num; ++i) {
    cout << i << " ";
    boards.push_back( apbGen.generateImageOfBoard(background, camMat, distCoeffs) );
  }
  cout << " Done" << endl;

  //vector<Point3f> chessboard3D = apbGen.worldPoints();

  /* init points */
  vector< vector<Point3f> > objectPoints;
  vector< vector<Point2f> > imagePoints;

  cout << "Finding April tags..." << endl;
  for(size_t i = 0; i < brds_num; ++i)
  {
    cout << "Board " << i << ": ";
    namedWindow( mainWindow); 
    imshow(mainWindow, boards[i].image);
    //waitKey(10);

    Mat greyscale( boards[i].image.size(), CV_8UC1 );
    cvtColor( boards[i].image, greyscale, CV_BGR2GRAY );

    vector<AprilTags::TagDetection> detections = tagDetector.extractTags(greyscale);
    cout << "found " << detections.size() << " tags:" << endl;

    AprilTagDetectionSet set( detections );
    cout << "gridded " << set.gridCount() << endl;

    // Convert detections to contours to draw
    // Draw the current image with detections
    vector< vector<Point > > detectionOutlines;
    for (int d=0; d < detections.size(); d++) {
      //  cout << "Id " << detections[i].id << ": " << std::hex << detections[i].code << std::dec << endl;

      vector<Point> detectionCorners(4);
      for( int j = 0; j < 4; ++j ) {
        detectionCorners[j] = Point( detections[d].p[j].first, detections[d].p[j].second );
      }
      detectionOutlines.push_back( detectionCorners );
    }
    drawContours(boards[i].image, detectionOutlines, -1, Scalar(0,0,255), 4, CV_AA);


    if( detections.size() > 5 ) {

      Mat valid;
      Mat positions = apb.mostLikelyAlignment( set.gridOfIds(), valid );

      vector<Point3f> worldPts;
      vector<Point2f> imagePts;

      for( int y = 0; y < valid.rows; ++y ) 
        for( int x = 0; x < valid.cols; ++x  ) 
          if( valid.at<uint8_t>(y,x) > 0 ) {
            Point3f world( positions.at<Point3f>(y,x) );

            if( set.validAt( x, y ) ) {
              Point2f image( set.at(x,y).cxy.first, set.at(x,y).cxy.second );

              worldPts.push_back( world );
              imagePts.push_back( image );
            }
          }



      objectPoints.push_back( worldPts );
      imagePoints.push_back( imagePts );

      vector<Point2f> inImage = boards[i].boardToImage( worldPts, camMat, distCoeffs );

      for( int j = 0; j < inImage.size(); ++j ) {
        cv::circle( boards[i].image, inImage[j], 5, Scalar(255,255,0), -1 );
      }

      // print out each detection
      //for (int i=0; i<detections.size(); i++) {
      //  print_detection(detections[i]);
      //}


      //            objectPoints.push_back(chessboard3D);
      //
    } else {
      cout<< " not enough.";
    }

    imshow(mainWindow, boards[i].image); 
    waitKey(0);

    cout << endl;

  }
  cout << "Done" << endl;
  cvDestroyAllWindows();

  Mat camMat_est;
  camMat.copyTo( camMat_est ); // ensure this is a deep copy, not a shallow reference copy

  Mat distCoeffs_est;
  vector<Mat> rvecs, tvecs;

  cout << "Calibrating...";
  int flags = CV_CALIB_USE_INTRINSIC_GUESS;
  TermCriteria criteria( TermCriteria::COUNT+TermCriteria::EPS, 200, DBL_EPSILON ); 

  double rep_err = calibrateCamera(objectPoints, imagePoints, imgSize, 
      camMat_est, distCoeffs_est, rvecs, tvecs, flags, criteria );
  cout << "Done" << endl;

  //cout << endl << "Average Reprojection error: " << rep_err/brds_num/apbGen.cornersSize().area() << endl;
  cout << "==================================" << endl;
  cout << "Original camera matrix:\n" << camMat << endl;
  cout << "Original distCoeffs:\n" << distCoeffs << endl;
  cout << "==================================" << endl;
  cout << "Estimated camera matrix:\n" << (Mat_<double>&)camMat_est << endl;
  cout << "Estimated distCoeffs:\n" << (Mat_<double>&)distCoeffs_est << endl;

  return 0;
}




