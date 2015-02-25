
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <AprilTags/TagFamily.h>
#include <AprilTags/Tag16h5.h>

#include "april_tag_board_generator.h"

using namespace cv;
using namespace std;


AprilTagBoard::AprilTagBoard( const AprilTags::TagCodes &tagCode,
    const Size& _arraySize, 
    float tagSize, float tagSpacing )  
: _arraySize( _arraySize ), 
  _tagSize( tagSize ),
  _tagSpacing( tagSpacing ),
  _tagFamily( tagCode ),
  _tags( _arraySize, CV_16U )
{
  cv::randu( _tags, 0, _tagFamily.codes.size() );

  for( int i = 0; i < arraySize().width; ++i )
    for( int j = 0; j < arraySize().height; ++j )
      cout << i << ' ' << j << ": " << codeIdAt(i,j) << ' ' << std::hex << codeAt(i,j) << std::dec << endl;
}

//vector<Point3f> worldPoints( void )
//{
//  vector<Point3f> worldPts;
//  for(int j = 0; j < _patternSize.height; ++j)
//    for(int i = 0; i < _patternSize.width; ++i)
//      chessboard3D.push_back(Point3f(i*_tagSpacing, j*_tagSpacing, 0));
//}

unsigned int AprilTagBoard::codeIdAt( int i, int j ) const
{
  assert( i >= 0 && i < arraySize().width &&
      j >= 0 && j < arraySize().height );
  return _tags.at<uint16_t>(j,i);
}

unsigned long long AprilTagBoard::codeAt( int i, int j ) const
{
  assert( i >= 0 && i < arraySize().width &&
      j >= 0 && j < arraySize().height );
  return _tagFamily.codes[ codeIdAt(i,j) ];
}

bool AprilTagBoard::hasId( unsigned int id ) const
{
  for( int i = 0; i < arraySize().width; ++i )
    for( int j = 0; j < arraySize().height; ++j )
    if( codeIdAt(i,j) == id ) return true;

  return false;
}

bool AprilTagBoard::idLocation( unsigned int id, Point3f &pt ) const
{
  for( int i = 0; i < arraySize().width; ++i )
    for( int j = 0; j < arraySize().height; ++j )
      if( codeIdAt(i,j) == id ) {
        pt = Point3f( margin() + tagSpacing()*i + 0.5 * tagSize(),
                    ( margin() + tagSpacing()*j ) + 0.5 * tagSize(), 0 );
      }


  return false;
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
    Point3f out(_origin + (scaled.x * _pb1) + (scaled.y * _pb2));

    //cout << "p: " << p << endl;
    //cout << "out: " << out << endl;

    return out;
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

struct StripZAxis
{
  Point2f operator()(const Point3f &p ) const
  { return Point2f( p.x, p.y ); }
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

vector<Point2f> SimulatedImage::boardToImage( const vector<Point3f> &boardPts, const Mat &camMat, const Mat &distCoeffs ) const
{
  Mat tvec(Mat::zeros(1, 3, CV_32F)), rvec;
  Rodrigues(Mat::eye(3, 3, CV_32F), rvec);

  vector <Point2f> boardPts2d;
  std::transform( boardPts.begin(), boardPts.end(), back_inserter( boardPts2d ),
      StripZAxis() );

  vector <Point3f> worldPts;
  vector <Point2f> imagePts;

    std::transform( boardPts2d.begin(), boardPts2d.end(),
        back_inserter( worldPts ), BoardToWorld( _origin, _pb1, _pb2 ) );
  projectPoints( Mat(worldPts), rvec, tvec, camMat, distCoeffs, imagePts);

  return imagePts;
}


//===========================================================================

const double AprilTagBoardGenerator::_minCos = 0.85; // 0.707;
const double AprilTagBoardGenerator::_cov = 0.8;
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
  pb2 = Point3f(b2[0]/len_b2, b2[1]/len_b2, b2[2]/len_b2);
}

Mat cv::AprilTagBoardGenerator::drawBoard(const Mat& bg, const Mat& camMat, const Mat& distCoeffs,
    const Point3f& origin, const Point3f& pb1, const Point3f& pb2 ) const
{

    // origin is 3D point for the center of the board in world coords
    // boardOrigin is the location of the upper left corner
    // of the board.
    //const Point3f boardOrigin( origin - pb1 * (_board.boardSize().width/2.0) + pb2 * (_board.boardSize().height/2.0) );
    //cout << boardOrigin << endl;
    const Point3f boardOrigin( origin );

  vector< vector<Point2f> > tagPixels;
  vector< Point2f > tagOrigins;
  for( int i = 0; i < _board.arraySize().width; ++i ) {
    for( int j = 0; j < _board.arraySize().height; ++j ) {
      //Point2f tagOrigin( _board.margin() + i * _board.tagSpacing() - (0.5 * _board.tagSize()), 
      //                   _board.margin() + j * _board.tagSpacing() - (0.5 * _board.tagSize()) );
      Point2f tagOrigin( _board.margin() + i * _board.tagSpacing(),
                         _board.margin() + j * _board.tagSpacing() );

      tagOrigins.push_back( tagOrigin );

      unsigned long long code = _board.codeAt( i, j );

      // First, build a bitmap of each tag 
      Mat bitmap( Mat::ones( Size( _board.tagSizePixels(), _board.tagSizePixels() ), CV_8U ) );
      Mat roi( bitmap, Rect( 1,1, _board.tagDimension(), _board.tagDimension() ) );

      // Bits in codeword (for 16 at least)
      // with 1 bit indicated _white_, otherwise _black_
      //
      // In the "canonical" tag images, starts with LSB at lower _right_ and goes up?
      // This works in canonical image coordinates, origin at upper left and goes
      // down ... they're the same just rotated by 180 
      for( int y = 0; y < _board.tagDimension(); ++y )
        for( int x = 0; x < _board.tagDimension(); ++x ) 
          if( code & (1 << (y*_board.tagDimension() + x)) )
            roi.at<uint8_t>(x,y) = 0;


      for( int x = 0; x < _board.tagSizePixels(); ++x ) 
        for( int y = 0; y < _board.tagSizePixels(); ++y ) 
          if( bitmap.at<uint8_t>(x,y) != 0 ) {
            vector<Point2f> corners;
            Point2f bitOrigin = tagOrigin + Point2f( x * _board.pixelSize(), y * _board.pixelSize() );
            tagPixels.push_back( generateCorners( bitOrigin, Point2f(_board.pixelSize(), _board.pixelSize() ) ) );
          }

    }

  }

  // Process all of the board-frame contours into world-frame contours
  vector< vector<Point > > tagContours;
  for( vector< vector<Point2f> >::iterator itr = tagPixels.begin(); itr != tagPixels.end(); ++itr ) {

    vector< Point3f > tagContourWorld;
    std::transform( (*itr).begin(), (*itr).end(),
        back_inserter(tagContourWorld),
        BoardToWorld(boardOrigin, pb1, pb2) );

    tagContours.push_back( worldToImage( tagContourWorld, camMat, distCoeffs ) );
  }


  vector< Point3f > tagOriginsWorld;
  std::transform( tagOrigins.begin(), tagOrigins.end(), back_inserter(tagOriginsWorld),
      BoardToWorld( boardOrigin, pb1, pb2) );
  vector< Point2f > tagOriginsImage;
  projectPoints( Mat(tagOriginsWorld), rvec, tvec, camMat, distCoeffs, tagOriginsImage );

  // -- Draw the outline of the board --
  vector <Point2f> boardCorners(4);
  boardCorners[0] = Point2f( 0, 0 );
  boardCorners[1] = Point2f( _board.boardSize().width, 0 );
  boardCorners[2] = Point2f( _board.boardSize().width, _board.boardSize().height );
  boardCorners[3] = Point2f( 0, _board.boardSize().height );

  vector <Point3f> boardCornersWorld;
  std::transform( boardCorners.begin(), boardCorners.end(), back_inserter( boardCornersWorld ),
      BoardToWorld( boardOrigin, pb1, pb2) );

  vector< vector<Point > > outlineContour;
  outlineContour.push_back( worldToImage( boardCornersWorld, camMat, distCoeffs ) );

  //  cout << "origin: " << origin << endl;
  //  cout << "pb1: " << pb1 << endl;
  //  cout << "pb2: " << pb2 << endl;
  //cout << "boardSize: " << boardSize << endl;
  //cout << "board.boardSize: " << _board.boardSize() << endl;
  //cout << "board.boardAspectRatio: " << _board.boardAspectRatio() << endl;

  //cout << "Board corner 0: " << boardCorners[0] << endl;
  //cout << "Board corner 1: " << boardCorners[1] << endl;
  //cout << "Board corner 2: " << boardCorners[2] << endl;
  //cout << "Board corner 3: " << boardCorners[3] << endl;

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

#define DRAW_COORDINATE_SYSTEM
#ifdef DRAW_COORDINATE_SYSTEM
    // Draw the coordinate system
    vector< Point3f > worldOrigins(3);
    const float axisLength = 1;
    worldOrigins[0] = origin;
    worldOrigins[1] = origin + pb1 * axisLength;
    worldOrigins[2] = origin + pb2 * axisLength;
    vector< Point2f > imageOrigins;
    projectPoints( Mat(worldOrigins), rvec, tvec, camMat, distCoeffs, imageOrigins );

    for( int i = 0; i < 3; ++i ) 
      imageOrigins[i] *= _rendererResolutionMultiplier;

    cv::circle( tmp, imageOrigins[0], 4*_rendererResolutionMultiplier, Scalar(0,0,255 ), -1 );
    cv::line( tmp, imageOrigins[0], imageOrigins[1],
        Scalar(255,0,0), 4*_rendererResolutionMultiplier );
    cv::line( tmp, imageOrigins[0], imageOrigins[2],
        Scalar(0,255,0), 4*_rendererResolutionMultiplier );
#endif

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

SimulatedImage cv::AprilTagBoardGenerator::generateImageOfBoard(const Mat& bg, const Mat& camMat, const Mat& distCoeffs ) const
{
  Point2f fov = fieldOfView( camMat, bg.size() );

  RNG& rng = theRNG();

  // Apparent half width/height
  // At unit distance (norm(p)=1), the board is half the FOV wide
  // Then the height is set by aspect ratio
  //float cbHalfWidth = static_cast<float>(norm(p) * sin( min(fov.x, fov.y) * 0.5 * CV_PI / 180));
  //float cbHalfHeight = cbHalfWidth * _board.boardAspectRatio();
  float cbHalfWidth = _board.boardSize().width / 2.0;
  float cbHalfHeight = _board.boardSize().height / 2.0;

  // Generate basis vectors for the board
    Point3f p;
  Point3f pb1, pb2;

  for(;;) {
    // Randomized distance from camera, "azimuth" (angle in the horizontal axis)
    // and "elevation" (angle in the verical axis)
    float d1 = static_cast<float>(rng.uniform(1.0, 10.0));
    float ah = static_cast<float>(rng.uniform(-fov.x/2 * _cov, fov.x/2 * _cov) * CV_PI / 180);
    float av = static_cast<float>(rng.uniform(-fov.y/2 * _cov, fov.y/2 * _cov) * CV_PI / 180);

    // Generate point for center of board.
    p.z = cos(ah) * d1;
    p.x = sin(ah) * d1;
    p.y = p.z * tan(av);

    generateBasis(pb1, pb2);

    Point3f pb3( pb1.cross(pb2) );

    // The coordinate system of the image must point in the same direction
    // as the system of the camera
    // (that is, if +Z of image plane is pointing towards the camera, try again...
    if( pb3.z < 0 ) continue;


    vector<Point3f> pts3d(4);
    vector<Point2f> pts2d(4);

    // Project corners of the board in 3D
    pts3d[0] = p + pb1 * cbHalfWidth - cbHalfHeight * pb2;
    pts3d[1] = p + pb1 * cbHalfWidth + cbHalfHeight * pb2;
    pts3d[2] = p - pb1 * cbHalfWidth + cbHalfHeight * pb2;
    pts3d[3] = p - pb1 * cbHalfWidth - cbHalfHeight * pb2;

    /* can remake with better perf */
    projectPoints( Mat(pts3d), rvec, tvec, camMat, distCoeffs, pts2d);

    // Only accept if all four corners are visible?
    bool inrect1 = pts2d[0].x < bg.cols && pts2d[0].y < bg.rows && pts2d[0].x > 0 && pts2d[0].y > 0;
    bool inrect2 = pts2d[1].x < bg.cols && pts2d[1].y < bg.rows && pts2d[1].x > 0 && pts2d[1].y > 0;
    bool inrect3 = pts2d[2].x < bg.cols && pts2d[2].y < bg.rows && pts2d[2].x > 0 && pts2d[2].y > 0;
    bool inrect4 = pts2d[3].x < bg.cols && pts2d[3].y < bg.rows && pts2d[3].x > 0 && pts2d[3].y > 0;

    if ( inrect1 && inrect2 && inrect3 && inrect4)
      break;
  }

  // Define an origin in the upper left of the calibration image
    Point3f origin( p - pb1 * cbHalfWidth - cbHalfHeight * pb2 );

  return SimulatedImage( drawBoard(bg, camMat, distCoeffs, origin, pb1, pb2 ), 
      origin, pb1, pb2 );
}

