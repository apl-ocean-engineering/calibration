
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

//struct CompareReprojErrors {
//  public:
//    bool operator()( pair< double, Matrix3d > a, pair< double, Matrix3d> b )
//    {  return (a.first < b.first); }
//};


struct DetectionNode 
{
  public:
    DetectionNode( const TagDetection &tag )
      : detection( tag ), up( NULL ), down(NULL), left(NULL), right(NULL)
    {;}

    const TagDetection &detection;
    DetectionNode *up, *down, *left, *right;
};

bool CompareAlongTrack( const pair< double, DetectionNode *> &a, const pair< double, DetectionNode *> &b  )
{ 
  return (a.first < b.first);  
}

static const float cosLimit = cos( 15 * M_PI / 180.0 );
DetectionNode *nearestNode( const vector< pair< Vector2d, DetectionNode *> > &centers, const Vector2d &vector )
{
  map< double, DetectionNode * > candidates;

  for( int i = 0; i < centers.size(); ++i ) {
    double along = vector.dot( centers[i].first );
    double cos = along / (vector.norm() * centers[i].first.norm() );

    cout << "To " << centers[i].second->detection.id << " is " << along << " cos " << cos << endl;

    if( (cos > cosLimit ) and (along > 0) ) 
      candidates[ along ] = centers[i].second;

  }

  if( candidates.size() == 0 ) return NULL;

  // Want the smallest positive dot product
  return candidates.begin()->second;
}

void assignToGraph( const DetectionNode *node, Mat &graph, const int x, const int y )
{
  assert( graph.at<int16_t>(y,x) == -1 || graph.at<int16_t>(y,x) == node->detection.id );

  graph.at<int16_t>(y,x) = node->detection.id;

  if( node->left ) assignToGraph( node->left, graph, x-1, y );
  if( node->right ) assignToGraph( node->right, graph, x+1, y );
  if( node->up )   assignToGraph( node->up, graph, x, y+1 );
  if( node->down ) assignToGraph( node->down, graph, x, y-1 );
}

void AprilTagDetectionSet::filterByHomography( void )
{
  // Identify the four corners of the 
  vector< DetectionNode * > nodes( _detections.size() );

  //for( int select = 0; select < _detections.size(); ++select )
 for( int select = 0; select < _detections.size(); ++select )
    nodes[select] = new DetectionNode( _detections[select] );

  //for( int select = 0; select < nodes.size(); ++select ) {
  for( int select = 0; select < 1; ++select ) {
    // Warp all other tag centers to my tag space

    vector< pair< Vector2d, DetectionNode * >  > centers;
    Matrix3d invHom( nodes[select]->detection.homography.inverse() );

    for( int other = 0; other < nodes.size(); ++other ) {
      if( other == select ) continue;


      Vector3d warped( invHom *
          Vector3d( nodes[other]->detection.cxy.first - nodes[select]->detection.hxy.first,
            nodes[other]->detection.cxy.second - nodes[select]->detection.hxy.second, 1.0 ) );
          //Vector3d( nodes[other]->detection.cxy.first, nodes[other]->detection.cxy.second, 1.0 ) );

     Vector2d c( warped.x() / warped.z(), warped.y() / warped.z() );
    // pair<float,float> pt = nodes[select]->detection.interpolate( nodes[other]->detection.cxy.first, nodes[other]->detection.cxy.second );
    // Vector2d c( pt.first, pt.second );

      cout << "Relative to " << nodes[select]->detection.id << " the center of " << nodes[other]->detection.id << " is at " << c.x() << "   " << c.y() << endl;

      centers.push_back(  make_pair(c,nodes[other] ) );
    }

    // Now search in each of the four cardinal directions.  What does that mean?
    // How about smallest along-axis which is larger than the cross-axis

    if( !nodes[select]->right ) {
    cout << "Evaluate right " << endl;
      DetectionNode *plusX = nearestNode( centers, Vector2d( 1, 0 ) );
      if( plusX ) {
        cout << "Right of " << nodes[select]->detection.id << " is " << plusX->detection.id << endl;
        nodes[select]->right = plusX;
        //assert( !plusX->left );
        plusX->left = nodes[select];
      }
    }

    if( !nodes[select]->left ) {
      cout << "Evaluate left " << endl;
      DetectionNode *plusX = nearestNode( centers, Vector2d( -1, 0 ) );
      if( plusX ) {
        cout << "Left of " << nodes[select]->detection.id << " is " << plusX->detection.id << endl;
        nodes[select]->left = plusX;
        //assert( !plusX->right );
        plusX->right = nodes[select];
      }
    }

    if( !nodes[select]->up ) {
      cout << "Evaluate up" << endl;
      DetectionNode *plusX = nearestNode( centers, Vector2d( 0,1 ) );
      if( plusX ) {
        cout << "Above of " << nodes[select]->detection.id << " is " << plusX->detection.id << endl;
        nodes[select]->up = plusX;
        //assert( !plusX->down );
        plusX->down = nodes[select];
      }
    }

    if( !nodes[select]->down ) {
      cout << "Evaluate down" << endl;
      DetectionNode *plusX = nearestNode( centers, Vector2d( 0,-1 ) );
      if( plusX ) {
        cout << "Below of " << nodes[select]->detection.id << " is " << plusX->detection.id << endl;
        nodes[select]->down = plusX;
        //assert( !plusX->up );
        plusX->up = nodes[select];
      }
    }
  }

  return;

  // Now convert it to an Matrix of values.
Mat graph( Size(2 * _detections.size(), 2* _detections.size()), CV_16S );
graph.setTo( -1 );
int x = _detections.size(), y = _detections.size();

assignToGraph( nodes[0], graph, x, y );

// Find the bounding box of non-(-1) values
cout << graph << endl;


  for( int i = 0; i < nodes.size(); ++i )
    delete nodes[i];

}



#ifdef  FOOBAR
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

      // Offset relative to mean 
      Vector2d mean(0.0, 0.0);
      for( int i = 0; i < 4; ++i ) 
        mean += Vector2d( projCorners[i].x(), projCorners[i].y() );
      mean *= 1.0/4;

      vector<Vector2d> corners(4);
      for( int i = 0; i < 4; ++i ) {
        corners[i] = Vector2d( projCorners[i].x() - mean.x(), projCorners[i].y() - mean.y() );
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
        reprojError += ( reprojCorners[j][i] - meanCorner[i] ).squaredNorm();


    answers.push_back( make_pair( sqrt(reprojError), homography ) );

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
#endif
