
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <AprilTags/TagFamily.h>
#include <AprilTags/Tag16h5.h>

#include "april_tag_board_generator.h"

using namespace cv;
using namespace std;


AprilTagBoard::AprilTagBoard(const Size& _arraySize, 
    double tagSize, double tagSpacing )  
: _arraySize( _arraySize ), 
  _tagSize( tagSize ),
  _tagSpacing( tagSpacing ),
  _tagFamily( AprilTags::tagCodes16h5 ), 
  _tags( _arraySize, CV_16U )
{
  cv::randu( _tags, 0, _tagFamily.codes.size() );
}

//vector<Point3f> worldPoints( void )
//{
//  vector<Point3f> worldPts;
//  for(int j = 0; j < _patternSize.height; ++j)
//    for(int i = 0; i < _patternSize.width; ++i)
//      chessboard3D.push_back(Point3f(i*_tagSpacing, j*_tagSpacing, 0));
//}


unsigned long long AprilTagBoard::codeAt( int i, int j ) const
{
  assert( i >= 0 && i < arraySize().width &&
      j >= 0 && j < arraySize().height );
  return _tagFamily.codes[ j*arraySize().width + i ];
}


//===========================================================================
// Operators for std::transform
//===========================================================================

struct BoardToWorld
{
  const Point3f &_origin, &_pb1, &_pb2; 
  const Point2f &_scale;

  BoardToWorld( const Point3f &origin, const Point3f &pb1, const Point3f &pb2, const Point2f &scale = Point2f(1,1) )
    : _origin(origin), _pb1(pb1), _pb2(pb2), _scale(scale)
  {;}

  Point3f operator()( const Point2f &p ) const
  {
    Point2f scaled( p.x * _scale.x, p.y * _scale.y );
    return _origin + (scaled.x * _pb1) + (scaled.y * _pb2);
  }
};


struct Mult
{
  float m;

  Mult(int mult) 
    : m((float)mult) {}

  Point2f operator()(const Point2f& p) const 
  { return p * m; }
};


static vector<Point2f> generateCorners( const Point2f &origin, const Point2f &dxdy )
{
  vector<Point2f> c;
  c.push_back( origin );
  c.push_back( origin + Point2f( dxdy.x, 0 ) );
  c.push_back( origin + Point2f( dxdy.x, dxdy.y ) );
  c.push_back( origin + Point2f( 0, dxdy.y ) );
  return c;
}


//===========================================================================
//===========================================================================

const double AprilTagBoardGenerator::_minCos = 0.707;
const double AprilTagBoardGenerator::_cov = 0.5;
const int AprilTagBoardGenerator::_rendererResolutionMultiplier = 4;

const size_t AprilTagBoardGenerator::_segmentsPerEdge = 200;

AprilTagBoardGenerator::AprilTagBoardGenerator( const AprilTagBoard &board )
  : 
    tvec(Mat::zeros(1, 3, CV_32F)),
    _board( board )
{
  Rodrigues(Mat::eye(3, 3, CV_32F), rvec);
}

void cv::AprilTagBoardGenerator::generateEdge(const Point3f& p1, const Point3f& p2, vector<Point3f>& out) const
{
  Point3f step = (p2 - p1) * (1.f/_segmentsPerEdge);
  for(size_t n = 0; n < _segmentsPerEdge; ++n)
    out.push_back( p1 + step * (float)n);
}

vector<Point> AprilTagBoardGenerator::worldToImage( const vector<Point3f> &worldPts, 
    const Mat& camMat, const Mat& distCoeffs)  const
{
  vector<Point3f> edges3d;

  // worldPts is assumed to be a set of corners describing a closed contour
  if( worldPts.size() < 2 ) return vector<Point>();

  int i;
  for( i = 0; i < worldPts.size()-1; ++i ) {
    generateEdge(worldPts[i], worldPts[i+1], edges3d);
  }
  generateEdge(worldPts[i], worldPts[0], edges3d);

  vector<Point2f> edges2d;
  projectPoints( Mat(edges3d), rvec, tvec, camMat, distCoeffs, edges2d);

  vector<Point2f> edges2d_reduced;
  approxPolyDP(Mat(edges2d), edges2d_reduced, 1.0, true);

  vector< Point > renderEdges;
  std::transform(edges2d_reduced.begin(), edges2d_reduced.end(),
      back_inserter(renderEdges),
      Mult(_rendererResolutionMultiplier));

  return renderEdges;
}


void cv::AprilTagBoardGenerator::generateBasis(Point3f& pb1, Point3f& pb2) const
{
  RNG& rng = theRNG();

  Vec3f n;
  for(;;)
  {
    n[0] = rng.uniform(-1.f, 1.f);
    n[1] = rng.uniform(-1.f, 1.f);
    n[2] = rng.uniform(-1.f, 1.f);
    float len = (float)norm(n);
    n[0]/=len;
    n[1]/=len;
    n[2]/=len;

    if (fabs(n[2]) > _minCos)
      break;
  }

  Vec3f n_temp = n; n_temp[0] += 100;
  Vec3f b1 = n.cross(n_temp);
  Vec3f b2 = n.cross(b1);
  float len_b1 = (float)norm(b1);
  float len_b2 = (float)norm(b2);

  pb1 = Point3f(b1[0]/len_b1, b1[1]/len_b1, b1[2]/len_b1);
  pb2 = Point3f(b2[0]/len_b1, b2[1]/len_b2, b2[2]/len_b2);
}

Mat cv::AprilTagBoardGenerator::drawBoard(const Mat& bg, const Mat& camMat, const Mat& distCoeffs,
    const Point3f& origin, const Point3f& pb1, const Point3f& pb2,
    const Size2f &boardSize, 
    vector<Point2f>& corners) const
{
  //    vector< vector<Point> > squares_black; //    for(int i = 0; i < _board.arraySize().width; ++i)
  //        for(int j = 0; j < _board.arraySize().height; ++j)
  //            if ( (i % 2 == 0 && j % 2 == 0) || (i % 2 != 0 && j % 2 != 0) )
  //            {
  //                vector<Point3f> pts_square3d;
  //                vector<Point2f> pts_square2d;
  //
  //                Point3f p1 = zero + (i + 0) * sqWidth * pb1 + (j + 0) * sqHeight * pb2;
  //                Point3f p2 = zero + (i + 1) * sqWidth * pb1 + (j + 0) * sqHeight * pb2;
  //                Point3f p3 = zero + (i + 1) * sqWidth * pb1 + (j + 1) * sqHeight * pb2;
  //                Point3f p4 = zero + (i + 0) * sqWidth * pb1 + (j + 1) * sqHeight * pb2;
  //                generateEdge(p1, p2, pts_square3d);
  //                generateEdge(p2, p3, pts_square3d);
  //                generateEdge(p3, p4, pts_square3d);
  //                generateEdge(p4, p1, pts_square3d);
  //
  //                projectPoints( Mat(pts_square3d), rvec, tvec, camMat, distCoeffs, pts_square2d);
  //                squares_black.resize(squares_black.size() + 1);
  //                vector<Point2f> temp;
  //                approxPolyDP(Mat(pts_square2d), temp, 1.0, true);
  //                transform(temp.begin(), temp.end(), back_inserter(squares_black.back()), Mult(rendererResolutionMultiplier));
  //            }

  //    /* calculate corners */
  //    vector<Point3f> corners3d;
  //    for(int j = 0; j < _board.arraySize().height - 1; ++j)
  //        for(int i = 0; i < _board.arraySize().width - 1; ++i)
  //            corners3d.push_back(zero + (i + 1) * sqWidth * pb1 + (j + 1) * sqHeight * pb2);
  //
  //    corners.clear();
  //    projectPoints( Mat(corners3d), rvec, tvec, camMat, distCoeffs, corners);

  // Identify the April tag origins
  Point2f scale( boardSize.width / _board.boardSize().width,
                 boardSize.height / _board.boardSize().height );


  vector< vector<Point2f> > tagPixels;
  vector< Point2f > tagOrigins;
  for( int i = 0; i < _board.arraySize().width; ++i ) {
    for( int j = 0; j < _board.arraySize().height; ++j ) {
      Point2f tagOrigin( (i+1) * _board.tagSpacing() - (0.5 * _board.tagSize()), 
          (j+1) * _board.tagSpacing() - (0.5 * _board.tagSize()) );

      tagOrigins.push_back( tagOrigin );

      unsigned long long code = _board.codeAt( i, j );

      // First, build a bitmap of each tag 
      Mat bitmap( Mat::ones( Size( _board.tagSizePixels(), _board.tagSizePixels() ), CV_8U ) );
      Mat roi( bitmap, Rect( 1,1, _board.tagDimension(), _board.tagDimension() ) );
      roi.setTo( 0 );

      for( int x = 0; x < _board.tagDimension(); ++x ) {
        for( int y = 0; y < _board.tagDimension(); ++y ) {
          if( code & (1 << (y*_board.tagDimension() + x)) )
            roi.at<uint8_t>(x,y) = 1;
        }
      }


      for( int x = 0; x < _board.tagSizePixels(); ++x ) {
        for( int y = 0; y < _board.tagSizePixels(); ++y ) {
          if( bitmap.at<uint8_t>(x,y) != 0 ) {
            vector<Point2f> corners;
            Point2f bitOrigin = tagOrigin + Point2f( x * _board.pixelSize(), y * _board.pixelSize() );
            tagPixels.push_back( generateCorners( bitOrigin, Point2f(_board.pixelSize(), _board.pixelSize() ) ) );
          }
        }
      }





      cout << "Finished: " << endl << bitmap << endl;


    }
  }

  // Process all of the board-frame contours into world-frame contours
  vector< vector<Point > > tagContours;
  for( vector< vector<Point2f> >::iterator itr = tagPixels.begin(); itr != tagPixels.end(); ++itr ) {

    vector< Point3f > tagContourWorld;
    std::transform( (*itr).begin(), (*itr).end(),
        back_inserter(tagContourWorld),
        BoardToWorld(origin, pb1, pb2, scale) );

    tagContours.push_back( worldToImage( tagContourWorld, camMat, distCoeffs ) );
  }


  vector< Point3f > tagOriginsWorld;
  std::transform( tagOrigins.begin(), tagOrigins.end(), back_inserter(tagOriginsWorld),
      BoardToWorld( origin, pb1, pb2, scale ) );
  vector< Point2f > tagOriginsImage;
  projectPoints( Mat(tagOriginsWorld), rvec, tvec, camMat, distCoeffs, tagOriginsImage );

  // -- Draw the outline of the board --
  vector <Point3f> boardCorners(4);
  boardCorners[0] = origin;
  boardCorners[1] = origin + (pb1 * boardSize.width);
  boardCorners[2] = origin + (pb1 * boardSize.width) + (pb2 * boardSize.height);
  boardCorners[3] = origin + (pb2 * boardSize.height);

  //  cout << "origin: " << origin << endl;
  //  cout << "pb1: " << pb1 << endl;
  //  cout << "pb2: " << pb2 << endl;
  //  cout << "boardSize: " << boardSize << endl;
  //
  cout << "Board corner 0: " << boardCorners[0] << endl;
  cout << "Board corner 1: " << boardCorners[1] << endl;
  cout << "Board corner 2: " << boardCorners[2] << endl;
  cout << "Board corner 3: " << boardCorners[3] << endl;

  vector< vector<Point > > outlineContour;
  outlineContour.push_back( worldToImage( boardCorners, camMat, distCoeffs ) );

  // Draw everything to the canvas
  Mat result;
  if( _rendererResolutionMultiplier == 1 )
  {
    result = bg.clone();
    drawContours(result, outlineContour, -1, Scalar::all(255), CV_FILLED, CV_AA);
    drawContours(result, tagContours, -1, Scalar::all(0), CV_FILLED, CV_AA);

    ///for( vector< Point2f >::iterator itr = tagOriginsImage.begin(); 
    ///    itr != tagOriginsImage.end(); ++itr ) {
    ///  cv::circle( result, (*itr), 5, Scalar( 255,0,255 ), -1 );
    ///}
  }
  else
  {
    Mat tmp;
    resize(bg, tmp, bg.size() * _rendererResolutionMultiplier);
    drawContours(tmp, outlineContour, -1, Scalar::all(255), CV_FILLED, CV_AA);
    drawContours(tmp, tagContours, -1, Scalar::all(0), CV_FILLED, CV_AA);

    //for( vector< Point2f >::iterator itr = tagOriginsImage.begin(); 
    //    itr != tagOriginsImage.end(); ++itr ) {
    //  cv::circle( tmp, (*itr)*_rendererResolutionMultiplier, 5*_rendererResolutionMultiplier,
    //      Scalar( 255,0,255 ), -1 );
    //}


    resize(tmp, result, bg.size(), 0, 0, INTER_AREA);
  }

  return result;
}

Point2f cv::AprilTagBoardGenerator::fieldOfView( const Mat &camMat, const Size &imgSize ) const
{
  const double sensorWidth = 32;
  const double sensorHeight = 24;
  double fovx, fovy, focalLen;
  Point2d principalPoint;
  double aspect;

  calibrationMatrixValues( camMat, imgSize, sensorWidth, sensorHeight,
      fovx, fovy, focalLen, principalPoint, aspect);

  return Point2f( fovx, fovy );
}

Mat cv::AprilTagBoardGenerator::generateImageOfBoard(const Mat& bg, const Mat& camMat, const Mat& distCoeffs, vector<Point2f>& corners) const
{
  Point2f fov = fieldOfView( camMat, bg.size() );

  RNG& rng = theRNG();

  // Randomized distance from camera, "azimuth" (angle in the horizontal axis)
  // and "elevation" (angle in the verical axis)
  float d1 = static_cast<float>(rng.uniform(0.1, 10.0));
  float ah = static_cast<float>(rng.uniform(-fov.x/2 * _cov, fov.x/2 * _cov) * CV_PI / 180);
  float av = static_cast<float>(rng.uniform(-fov.y/2 * _cov, fov.y/2 * _cov) * CV_PI / 180);

  // Generate point for center of board.
  Point3f p;
  p.z = cos(ah) * d1;
  p.x = sin(ah) * d1;
  p.y = p.z * tan(av);

  // Generate basis vectors for the board
  Point3f pb1, pb2;
  generateBasis(pb1, pb2);

  // Apparent half width/height
  // At unit distance (norm(p)=1), the board is half the FOV wide
  // Then the height is set by aspect ratio
  float cbHalfWidth = static_cast<float>(norm(p) * sin( min(fov.x, fov.y) * 0.5 * CV_PI / 180));
  float cbHalfHeight = cbHalfWidth * _board.boardAspectRatio();

  vector<Point3f> pts3d(4);
  vector<Point2f> pts2d(4);
  for(;;)
  {
    // Project corners of the board in 3D
    pts3d[0] = p + pb1 * cbHalfWidth + cbHalfHeight * pb2;
    pts3d[1] = p + pb1 * cbHalfWidth - cbHalfHeight * pb2;
    pts3d[2] = p - pb1 * cbHalfWidth - cbHalfHeight * pb2;
    pts3d[3] = p - pb1 * cbHalfWidth + cbHalfHeight * pb2;

    /* can remake with better perf */
    projectPoints( Mat(pts3d), rvec, tvec, camMat, distCoeffs, pts2d);

    // Only accept if all four corners are visible?
    bool inrect1 = pts2d[0].x < bg.cols && pts2d[0].y < bg.rows && pts2d[0].x > 0 && pts2d[0].y > 0;
    bool inrect2 = pts2d[1].x < bg.cols && pts2d[1].y < bg.rows && pts2d[1].x > 0 && pts2d[1].y > 0;
    bool inrect3 = pts2d[2].x < bg.cols && pts2d[2].y < bg.rows && pts2d[2].x > 0 && pts2d[2].y > 0;
    bool inrect4 = pts2d[3].x < bg.cols && pts2d[3].y < bg.rows && pts2d[3].x > 0 && pts2d[3].y > 0;

    if ( inrect1 && inrect2 && inrect3 && inrect4)
      break;

    // Otherwise shrink, maintaining aspect ratio
    cbHalfWidth*=0.8f;
    cbHalfHeight = cbHalfWidth * _board.boardAspectRatio();
  }

  // Define the board origin as one of the corners
  Point3f origin = p - (pb1 * cbHalfWidth) - (cbHalfHeight * pb2);
  Size2f   boardSize( 2 * cbHalfWidth, 2 * cbHalfHeight );

  return drawBoard(bg, camMat, distCoeffs, origin, pb1, pb2, boardSize,  corners);
}

