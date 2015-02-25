
#include <iomanip>

// AprilTags currently uses Eigen for fixed-size vectors and matrices
#include <Eigen/Core>


#include "april_tag_detection_set.h"

using namespace Eigen;
using namespace cv;
using namespace std;


  AprilTagDetectionSet::AprilTagDetectionSet( const vector<TagDetection> detections )
: _detections( detections )
{;}

struct CompareReprojErrors {
  public:
    bool operator()( pair< double, Matrix3d > a, pair< double, Matrix3d> b )
    {  return (a.first < b.first); }
};

void AprilTagDetectionSet::filterByHomography( void )
{
  // Try a RANSAC-like approach first

  vector< pair< double, Matrix3d > > answers;
  for( int select = 0; select < _detections.size(); ++select )
  {
    Matrix3d homography( _detections[select].homography );

    std::vector< vector<Vector2d> > reprojCorners;
    for( int other = 0; other < _detections.size(); ++other ) {
      if( other == select ) continue;

      Vector3d projCorners[4];
      for( int i = 0; i < 4; ++i ) {
        projCorners[i] = homography.inverse() * Vector3d( _detections[other].p[i].first, _detections[other].p[i].second, 1.0 );
        projCorners[i] = projCorners[i] / projCorners[i].z();
      }

      // Offset relative to mean and convert to 2-vectors
      Vector2d mean(0.0, 0.0);
      for( int i = 0; i < 4; ++i ) 
        mean += Vector2d( projCorners[i].x(), projCorners[i].y() );
      mean *= 1.0/4;

      vector<Vector2d> corners(4);
      cout << "Corners: " << endl;
      for( int i = 0; i < 4; ++i ) {
        corners[i] = Vector2d( projCorners[i].x() - mean.x(), projCorners[i].y() - mean.y() );
        cout << std::setw(8) << corners[i].x() << ' ' << corners[i].y() << endl;
      }

      reprojCorners.push_back( corners );
    }

    // Compute the mean of each corner
    Vector2d meanCorner[4];
      for( int i = 0; i < 4; ++i ) {
meanCorner[i] = Vector2d(0.,0.);
        for( int j = 0; j < reprojCorners.size(); ++j ) {
          meanCorner[i] += reprojCorners[j][i];
        }
        meanCorner[i] /= reprojCorners.size();
      }

      double reprojError = 0;
      for( int i = 0; i < 4; ++i ) 
        for( int j = 0; j < reprojCorners.size(); ++j ) 
          reprojError += ( reprojCorners[j][i] - meanCorner[i] ).norm();

    
      answers.push_back( make_pair( reprojError, homography ) );

  }

  std::sort( answers.begin(), answers.end(), CompareReprojErrors() );

  Matrix3d bestHom( answers.begin()->second );

  cout << "Reproj centers" << endl;
  vector< Vector2d > reprojCenters( _detections.size() );
  Vector2d minCorner, maxCorner;
  for( int i = 0; i < _detections.size(); ++i ) {
    Vector3d ans( bestHom.inverse() * Vector3d(_detections[i].cxy.first, _detections[i].cxy.second, 1.0) );
    reprojCenters[i] = Vector2d( ans.x() / ans.z(), ans.y() / ans.z() );

    if( i == 0 ) {
      minCorner = maxCorner = reprojCenters[i];
    } else {
      minCorner.x() = std::min( minCorner.x(), reprojCenters[i].x() );
      minCorner.y() = std::min( minCorner.y(), reprojCenters[i].y() );
      maxCorner.x() = std::max( maxCorner.x(), reprojCenters[i].x() );
      maxCorner.y() = std::max( maxCorner.y(), reprojCenters[i].y() );
    }

    cout << reprojCenters[i].x() << ' ' << reprojCenters[i].y()  << endl;
  }

  float scale = 700.0 / std::max( maxCorner.y() - minCorner.y(), maxCorner.x() - minCorner.x() );
  Vector2d offset = -minCorner;

  cout << "y: " <<  maxCorner.y() - minCorner.y() << endl << "x: " << maxCorner.x() - minCorner.x() << endl;
  cout << "foo" << endl << scale << endl << offset << endl;

  cout << "Points in tag space." << endl;
  Mat img(  Mat::zeros( 800, 800, CV_8UC3 ) );
  for( int i = 0; i < _detections.size(); ++i ) {
    Vector2d o( reprojCenters[i] + offset );
    Point2f pt( (o.x() * scale) + 25, (o.y()*scale) + 25 );

    cout << pt.x << ' ' << pt.y << endl;

    cv::circle( img, pt, 5, Scalar(255,0,255), -1 );
  }
  imshow( "reprojCorners", img );

}

