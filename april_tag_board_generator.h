
#include <opencv2/core/core.hpp>
#include <vector>

#include <AprilTags/TagFamily.h>

#ifndef __APRIL_TAG_BOARD_GENERATOR_H__
#define __APRIL_TAG_BOARD_GENERATOR_H__

using cv::Point3f;
using cv::Point2f;
using cv::Mat;
using cv::Size;
using std::vector;

using AprilTags::TagFamily;

namespace cv {


  class AprilTagBoard
  {
    public:

      AprilTagBoard( const AprilTags::TagCodes &tagCode,
          const Size &arraySize, const float tagSize = 1, 
          const float tagSpacing = 1.5 );

      Size arraySize( void ) const { return _arraySize; }
      size_t length() const { return _arraySize.area(); }

      // These are now in "real units"
      // when the board is rendered...
      float tagSpacing( void ) const { return _tagSpacing; }
      float margin( void ) const { return _tagSize * 0.5; }
      float tagSize( void ) const { return _tagSize; }
      float pixelSize( void ) const  { return _tagSize / tagSizePixels(); }

      Size2f boardSize( void ) const 
      { return Size2f( _tagSpacing * (arraySize().width-1) + _tagSize + (2*margin()),
                       _tagSpacing * (arraySize().height-1)  + _tagSize + (2*margin()) ); }


      unsigned int tagSizePixels( void ) const {
        return _tagFamily.dimension + 2*_tagFamily.blackBorder;
     }

     unsigned int tagBits( void ) const { return _tagFamily.bits; }
     unsigned int tagDimension( void ) const { return _tagFamily.dimension; }

     unsigned int codeIdAt( int i, int j ) const;
     unsigned long long codeAt( int i, int j ) const;

     Point3f locationAt( int x, int y ) const;

     bool hasId( unsigned int id ) const;
     bool idLocation( unsigned int id, Point3f &pt  ) const;

     Mat mostLikelyAlignment( const Mat &ids, Mat &valid );

    private:

    Size _arraySize;
    float _tagSize;
    float _tagSpacing;

    TagFamily _tagFamily;
    Mat _tags;

int calculateDistance( const Mat &ids, const Point2i &offset );

  };



  class SimulatedImage 
  {
    public:
      Mat image;

      SimulatedImage( const Mat &img, const Point3f &origin, const Point3f &pb1, const Point3f &pb2 )
        : image(img), _origin(origin), _pb1(pb1), _pb2(pb2) {;}

vector<Point2f> boardToImage( const vector<Point3f> &boardPts, const Mat &camMat, const Mat &distCoeffs ) const;

    private:

      Point3f _origin, _pb1, _pb2;
  };

class AprilTagBoardGenerator 
{
public:
    double sensorWidth;
    double sensorHeight;


    AprilTagBoardGenerator(const AprilTagBoard &board );

    // Generates a randomized pose for the board then renders an image
    // of the board against the background bg.
    //
    // Returns the rendered image.
    //
    SimulatedImage generateImageOfBoard(const Mat& bg, const Mat& camMat, const Mat& distCoeffs ) const;


private:

    // Calculate the field of view given the camera matrix, image size
    // and the sensor size (which is fixed)
    Point2f fieldOfView( const Mat &camMat, const Size &imgSize ) const;

    // Breaks a line segment into _segmentsPerEdge sub-segments
    void generateEdge(const Point3f& p1, const Point3f& p2, vector<Point3f>& out) const;

    vector<Point> worldToImage( const vector<Point3f> &worldPts, 
        const Mat& camMat, const Mat& distCoeffs)  const;

    Mat drawBoard(const Mat& bg, const Mat& camMat, const Mat& distCoeffs,
        const Point3f& origin, const Point3f& pb1, const Point3f& pb2 ) const;

    void generateBasis(Point3f& pb1, Point3f& pb2) const;

//    Point3f generateChessBoardCenter(const Mat& camMat, const Size& imgSize) const;


    const AprilTagBoard &_board;

    // Parameters for the randomized board generation
    static const double _minCos;
    static const double _cov;
    static const int _rendererResolutionMultiplier;

    static const size_t  _segmentsPerEdge;

    Mat rvec, tvec;
};

}

#endif
