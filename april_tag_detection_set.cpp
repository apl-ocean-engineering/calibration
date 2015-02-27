
#include <iomanip>
#include <fstream>

// AprilTags currently uses Eigen for fixed-size vectors and matrices
#include <Eigen/Core>

#include <gsl/gsl_randist.h>


#include "april_tag_detection_set.h"

using namespace Eigen;
using namespace cv;
using namespace std;


  AprilTagDetectionSet::AprilTagDetectionSet( const vector<TagDetection> detections )
: _detections( detections ), _grid()
{ 
  arrangeIntoGrid();
}

//struct CompareReprojErrors {
//  public:
//    bool operator()( pair< double, Matrix3d > a, pair< double, Matrix3d> b )
//    {  return (a.first < b.first); }
//};





cv::Mat AprilTagDetectionSet::gridOfIds( void )
{
  Mat ids( _grid.size(), CV_16S );

  for( int i = 0; i < _grid.rows; ++i ) 
    for( int j = 0; j < _grid.cols; ++j )  {
      int idx = _grid.at<int16_t>(i,j);
      ids.at<int16_t>(i,j) = (idx >= 0) ?  _detections[ _grid.at<int16_t>(i,j) ].id : -1;
    }

  return ids;
}


int AprilTagDetectionSet::gridCount( void ) const
{
  int count = 0;
  for( int y = 0; y < _grid.rows; ++y ) 
    for( int x = 0; x < _grid.cols; ++x )  
      if( indexAt(x,y) >= 0 ) ++count;

  return count;
}



//============================================================================
//
//
//============================================================================

bool CompareAlongTrack( const pair< double, int > &a, const pair< double, int> &b  )
{ 
  return (a.first < b.first);  
}

struct Gaussian
{
  public:
    Gaussian( float _mean, float _var )
      : mean( _mean ), sigma(  sqrt(_var) ) {;}

    double p( double val ) const
    { return gsl_ran_gaussian_pdf( val-mean, sigma ); }

    float mean, sigma;
};

static const float cosLimit = cos( 15 * M_PI / 180.0 );
int nearestNode( const vector< Vector2d > &centers, const Vector2d &vector )
{
  map< double, int > candidates;

  for( int i = 0; i < centers.size(); ++i ) {
    if( centers[i].hasNaN() ) continue;

    double along = vector.dot( centers[i] );
    double cos = along / (vector.norm() * centers[i].norm() );

    //cout << "To " << centers[i].second->detection.id << " is " << along << " cos " << cos << endl;

    if( (cos > cosLimit ) and (along > 0) ) 
      candidates[ along ] = i;

  }

  if( candidates.size() == 0 ) return -1;

  // Want the closest
  return candidates.begin()->second;
}

int mostLikelyNode( const vector< Vector2d > &centers, const Gaussian &gaussian, const Vector2d &vector )
{
  map< double, int > candidates;

  for( int i = 0; i < centers.size(); ++i ) {
    if( centers[i].hasNaN() ) continue;

    double along = vector.dot( centers[i] );
    double cos = along / (vector.norm() * centers[i].norm() );

    //cout << "To " << centers[i].second->detection.id << " is " << along << " cos " << cos << endl;

    float prob = gaussian.p(along);
    if( (cos > cosLimit ) and (along > 0) and (prob > 0.99) ) 
      candidates[ prob ] = i;

  }

  if( candidates.size() == 0 ) return -1;

  // Want the largest probability
  return candidates.rbegin()->second;
}

void AprilTagDetectionSet::assignToGraph( const vector<DetectionNode> &nodes, const int idx, Mat &graph, const int x, const int y, int *limits )
{
  //assert( graph.at<int16_t>(y,x) == -1 || graph.at<int16_t>(y,x) == node->detection.id );

  if( graph.at<int16_t>(y,x) != -1 ) return;
  if( y < 0 || x < 0 || y >= graph.rows || x >= graph.cols ) return;
  if( idx < 0 || idx >= _detections.size() ) return;

  //cout << "Assign " << node->detection.id << endl;

  graph.at<int16_t>(y,x) = idx;

  limits[0] = std::min( x, limits[0] );
  limits[1] = std::max( x, limits[1] );
  limits[2] = std::min( y, limits[2] );
  limits[3] = std::max( y, limits[3] );

  if( nodes[idx].left > 0 ) assignToGraph( nodes, nodes[idx].left, graph, x-1, y, limits );
  if( nodes[idx].right > 0 ) assignToGraph( nodes, nodes[idx].right, graph, x+1, y, limits );
  if( nodes[idx].up > 0 )   assignToGraph( nodes, nodes[idx].up, graph, x, y+1, limits );
  if( nodes[idx].down > 0 ) assignToGraph( nodes, nodes[idx].down, graph, x, y-1, limits );
}

Gaussian estimateCenterSpacing( list<double> &values )
{
  if( values.size() > 10 ) {
    // For robustness drop the largest and the smallest.
    values.sort();
    values.pop_front();
    values.pop_back();
  }

  // Assume inliers >> outliers for now

  double mean, var;

  for( list<double>::iterator itr = values.begin(); itr != values.end(); ++itr ) mean += (*itr);
  mean /= values.size();

  // Calculate sample variance
  for( list<double>::iterator itr = values.begin(); itr != values.end(); ++itr ) var += ( (*itr) - mean )*( (*itr) - mean );
  var /= (values.size() - 1);

  return Gaussian( mean, var );
}

void AprilTagDetectionSet::arrangeIntoGrid( void )
{
  // Identify the four corners of the 
  vector< DetectionNode > nodes( _detections.size() );

  // Try to identify the distribution of distances to adjacent tags
  list< double > lrSpacing, udSpacing;

  for( int current = 0; current < _detections.size(); ++current ) {

    Matrix3d invHom( _detections[current].homography.inverse() );
    vector< Vector2d  > centers( _detections.size() );

    centers[current] = Vector2d(NAN,NAN);

    for( int other = 0; other < _detections.size(); ++other ) {
      if( other == current ) continue;

      Vector3d warped( invHom *
          Vector3d( _detections[other].cxy.first - _detections[current].hxy.first,
            _detections[other].cxy.second - _detections[current].hxy.second, 1.0 ) );

      Vector2d c( warped.x() / warped.z(), warped.y() / warped.z() );

      //cout << "Relative to " << nodes[current]->detection.id << " the center of " << nodes[other]->detection.id << " is at " << c.x() << "   " << c.y() << endl;

      centers[other] = c;
    }

    int right = nearestNode( centers, Vector2d( 1, 0 ) );
    if( right >= 0 ) lrSpacing.push_back( centers[right].norm() );

    int left = nearestNode( centers, Vector2d( 1, 0 ) );
    if( left >= 0 ) lrSpacing.push_back( centers[left].norm() );

    int up = nearestNode( centers, Vector2d( 1, 0 ) );
    if( up >= 0 ) udSpacing.push_back( centers[up].norm() );

    int down = nearestNode( centers, Vector2d( 1, 0 ) );
    if( down >= 0 ) udSpacing.push_back( centers[down].norm() );

  }

  Gaussian lrGaussian = estimateCenterSpacing( lrSpacing );
  cout << "LR spacing has mean " << lrGaussian.mean << " sigma " << lrGaussian.sigma << endl;
  Gaussian udGaussian = estimateCenterSpacing( udSpacing );
  cout << "UD spacing has mean " << udGaussian.mean << " sigma " << udGaussian.sigma << endl;


  for( int current = 0; current < _detections.size(); ++current ) {

    Matrix3d invHom( _detections[current].homography.inverse() );
    vector< Vector2d  > centers( _detections.size() );

    centers[current] = Vector2d(NAN,NAN);

    for( int other = 0; other < _detections.size(); ++other ) {
      if( other == current ) continue;

      Vector3d warped( invHom *
          Vector3d( _detections[other].cxy.first - _detections[current].hxy.first,
            _detections[other].cxy.second - _detections[current].hxy.second, 1.0 ) );

      Vector2d c( warped.x() / warped.z(), warped.y() / warped.z() );

      //cout << "Relative to " << nodes[current]->detection.id << " the center of " << nodes[other]->detection.id << " is at " << c.x() << "   " << c.y() << endl;

      centers[other] = c;
    }

    if( nodes[current].right < 0 ) {
      //cout << "Evaluate right " << endl;
      int nearest = mostLikelyNode( centers, lrGaussian, Vector2d( 1, 0 ) );
      if( nearest >= 0 ) {
        //cout << "Right of " << _detections[current].id << " is " << _detections[nearest].id << endl;
        nodes[current].right = nearest;
        nodes[nearest].left = current;
      }
    }

    if( nodes[current].left < 0 ) {
      int nearest = mostLikelyNode( centers, lrGaussian, Vector2d( -1, 0 ) );
      if( nearest >= 0 ) {
        //cout << "Left of " << _detections[current].id << " is " << _detections[nearest].id << endl;
        nodes[current].left = nearest;
        nodes[nearest].right = current;
      }
    }

    if( nodes[current].up < 0 ) {
      int nearest = mostLikelyNode( centers, udGaussian, Vector2d( 0, 1 ) );
      if( nearest >= 0 ) {
        //cout << "Up of " << _detections[current].id << " is " << _detections[nearest].id << endl;
        nodes[current].up = nearest;
        nodes[nearest].down = current;
      }
    }

    if( nodes[current].down < 0 ) {
      int nearest = mostLikelyNode( centers, udGaussian, Vector2d( 0, -1 ) );
      if( nearest >= 0 ) {
        //cout << "Down of " << _detections[current].id << " is " << _detections[nearest].id << endl;
        nodes[current].down = nearest;
        nodes[nearest].up = current;
      }
    }
  }

  // Dump the graph to DOT to examine it
  ofstream dot("/tmp/graph.dot");

  dot << "graph april {" << endl;
  for( int i = 0; i < _detections.size(); ++i ) {
    dot << i << " [label=\"" << _detections[i].id << "\"]" << endl;
    if( nodes[i].right >= 0 ) dot << i << "--" << nodes[i].right << endl;
    if( nodes[i].left >= 0 ) dot << i << "--" << nodes[i].left << endl;
    if( nodes[i].up >= 0 ) dot << i << "--" << nodes[i].up << endl;
    if( nodes[i].down >= 0 ) dot << i << "--" << nodes[i].down << endl;
  }
  dot << "}" << endl;

  // Now convert it to an Matrix of values.
  Mat graph( Size(2 * _detections.size(), 2* _detections.size()), CV_16S );
  graph.setTo( -1 );
  int x = _detections.size(), y = _detections.size();

  int limits[4] = { x,x,y,y };

  vector< int > connections(4);
  for( int i = 0; i < 4; ++i ) connections[i] = -1;
  for( int i = 0; i < _detections.size(); ++i ) {
    int neighbors = 0;
    if( nodes[i].right >= 0 ) neighbors++;
    if( nodes[i].left >= 0 ) neighbors++;
    if( nodes[i].up >= 0 ) neighbors++;
    if( nodes[i].down >= 0 ) neighbors++;

    connections[neighbors] = i;

    if( neighbors == 4 ) break;
  }

  int use = connections[4];
  if( use == -1 ) use = connections[3];
if( use == -1 ) use = connections[2];
if( use == -1 ) { cout << "Tag graph is pathological.  Giving up" << endl; return; }

  assignToGraph( nodes, use, graph, x, y, limits );

  //cout << limits[0] << ' ' << limits[1] << ' ' << limits[2] << ' ' << limits[3] << endl;
  // Find the bounding box of non-(-1) values
  Rect rect( limits[0], limits[2], limits[1]-limits[0]+1, limits[3]-limits[2]+1 );
  Mat roi( graph, rect );

  _grid.create( roi.size(), roi.type() );

  // Can't figure out my axis problem that lead to this needing to be flipped...
  cv::flip( roi, _grid, 1 );

  //cout << _grid << endl;

  cout <<  "Identified IDs: " << endl << gridOfIds() << endl;

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
