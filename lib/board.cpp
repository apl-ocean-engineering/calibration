
#include <iostream>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "board.h"
#include "detection.h"

using namespace std;
using namespace cv;

Detection *Board::detectPattern( const Mat &gray, vector< Point2f > &pointbuf )
{
  Detection *detect = new Detection();

  switch( pattern )
  {
    case CHESSBOARD:
      detect->found = findChessboardCorners( gray, size(), detect->points,
          CV_CALIB_CB_ADAPTIVE_THRESH | CV_CALIB_CB_FAST_CHECK | CV_CALIB_CB_NORMALIZE_IMAGE);

      // improve the found corners' coordinate accuracy
      if( detect->found) cornerSubPix( gray, detect->points, Size(11,11), Size(-1,-1), TermCriteria( CV_TERMCRIT_EPS+CV_TERMCRIT_ITER, 30, 0.1 ));

      break;
    case CIRCLES_GRID:
      detect->found = findCirclesGrid( gray, size(), detect->points );
      break;
    case ASYMMETRIC_CIRCLES_GRID:
      detect->found = findCirclesGrid( gray, size(), detect->points, CALIB_CB_ASYMMETRIC_GRID );
      break;
    default:
      cerr << "Unknown pattern type" << endl;
      return NULL;
  }

  detect->calculateCorners( *this );
  return detect;
}


Board *Board::load( const string &infile, const string &name )
{
  FileStorage fs( infile, FileStorage::READ );
  if( ! fs.isOpened() ) {
    cout << "Couldn't open board file \"" << infile << "\"" << endl;
    exit(-1);
  }

  string type_s;
  int width, height;
  float squares;
  Pattern type;

  fs["type"] >> type_s;
  fs["width"] >> width;
  fs["height"] >> height;
  fs["squareSize"] >> squares;

  Board *board = NULL;
  if( type_s.compare("chessboard" ) == 0 ) {
    board = new Board( CHESSBOARD, width, height, squares, name );
  } else if( type_s.compare("apriltags_36h11" ) == 0) {
#ifdef USE_APRILTAGS
    board = new AprilTagsBoard( width, height, squares, name );
#else
    cout << "Not compiled for Apriltags." << endl;
#endif
  } else {
    cout << "Don't know how to handle board type \"" << type_s << "\"" << endl;
  }

  board->loadCallback( fs );

  return board;
}

//===========================================================================
//  AprilTagsBoard
//===========================================================================

//const int AprilTagsBoard::_ids[] = { 0,1,2,3,4,5,6,
//  24,25,26,27,28,29,30,
//  48,49,50,51,52,53,54,
//  72,73,74,75,76,77,78,
//  96,97,98,99,100,101,102 };

void AprilTagsBoard::loadCallback( FileStorage &fs )
{
  fs["ids"] >> _ids;
}

Detection *AprilTagsBoard::detectPattern( const cv::Mat &gray, vector< cv::Point2f > &pointbuf )
{
  AprilTags::TagDetector tagDetector( _tagCode );

  vector<AprilTags::TagDetection> detections = tagDetector.extractTags(gray);
  //cout << "found " << detections.size() << " tags:" << endl;

  AprilTagsDetection *detect = new AprilTagsDetection( detections );
  detect->calculateCorners( *this );
  return detect;
}

bool AprilTagsBoard::find( const int id, cv::Point2i &xy  ) const
{
  for( int x = 0; x < width; ++x ) 
    for( int y = 0; y < height; ++y ) 
      if( _ids.at<int>(y,x) == id ) 
      {
        xy.x = x; xy.y = y;
        return true;
      }

  return false;
}

cv::Point3f AprilTagsBoard::worldLocation( const cv::Point2i &xy ) const
{
  return Point3f( xy.x * squareSize, xy.y * squareSize, 0 );
}

