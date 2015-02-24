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



const Size imgSize(800, 600);
const Size brdSize(2,1);
const size_t brds_num = 1;

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

  cout << "Initializing April board..." << endl;
  AprilTagBoard apb( brdSize );

  AprilTagBoardGenerator apbGen( apb );
  cout << "Done" << endl;

  // Initialize camera parameters
  Mat_<double> camMat(3, 3);
  camMat << 300., 0., background.cols/2., 0, 300., background.rows/2., 0., 0., 1.;

  Mat_<double> distCoeffs(1, 5);
  distCoeffs << 1.2, 0.2, 0., 0., 0.;

  cout << "Generating chessboards...";
  vector<Mat> boards(brds_num);
  vector<Point2f> tmp;
  for(size_t i = 0; i < brds_num; ++i)
    cout << (boards[i] = apbGen.generateImageOfBoard(background, camMat, distCoeffs, tmp), i) << " ";
  cout << "Done" << endl;

  //vector<Point3f> chessboard3D = apbGen.worldPoints();

  /* init points */
  vector< vector<Point3f> > objectPoints;
  vector< vector<Point2f> > imagePoints;

  cout << endl << "Finding chessboards' corners...";
  for(size_t i = 0; i < brds_num; ++i)
  {
    cout << "Board " << i << ": ";
    namedWindow( mainWindow); 
    imshow(mainWindow, boards[i]);
    waitKey(0);

    vector<AprilTags::TagDetection> detections = tagDetector.extractTags(boards[i]);

    cout << "found " << detections.size();

    // Draw the current image with detections
    for (int i=0; i<detections.size(); i++) {
      detections[i].draw(boards[i]);
    }
    imshow(mainWindow, boards[i]); 
    waitKey(1000);

    if( detections.size() > 5 ) {
      // print out each detection
      cout << detections.size() << " tags detected:" << endl;
      //for (int i=0; i<detections.size(); i++) {
      //  print_detection(detections[i]);
      //}


      //            objectPoints.push_back(chessboard3D);
      //
    } else {
      cout<< " not enough.";
    }

    cout << endl;

    //        if (found)
    //        {
    //            imagePoints.push_back(tmp);
    //            objectPoints.push_back(chessboard3D);
    //            cout<< "-found ";
    //        }

    //        drawChessboardCorners(boards[i], apbGen.cornersSize(), Mat(tmp), found);
  }
  cout << "Done" << endl;
  cvDestroyAllWindows();

  return 0;

  Mat camMat_est;
  Mat distCoeffs_est;
  vector<Mat> rvecs, tvecs;

  cout << "Calibrating...";
  double rep_err = calibrateCamera(objectPoints, imagePoints, imgSize, camMat_est, distCoeffs_est, rvecs, tvecs);
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




