
#ifndef __DISTORTION_MODEL_H__
#define __DISTORTION_MODEL_H__

#include <opencv2/core/core.hpp>
#include <opencv2/core/affine.hpp>
#include <vector>

namespace Distortion {

  using std::vector;
  using cv::Size;
  using cv::Mat;

  using cv::Vec2f;
  using cv::Vec3f;
  using cv::Point3f;
  using cv::Point2f;

  using cv::Vec2d;
  using cv::Vec3d;

  using cv::Matx33d;

  // For later ... it's all done double precision for now.  Not necessary.

  typedef Vec3f ObjectPoint;
  typedef vector< ObjectPoint > ObjectPointsVec;
  typedef vector< vector< ObjectPoint > > ObjectPointsVecVec;

  typedef Vec2f ImagePoint;
  typedef vector< ImagePoint > ImagePointsVec;
  typedef vector< vector< ImagePoint > > ImagePointsVecVec;

  typedef vector< Vec3d > RotVec, TransVec;

  class Camera {
    public:

      virtual ~Camera() {;}

      virtual const std::string name( void ) const = 0;

      virtual double calibrate( const ObjectPointsVecVec &objectPoints, 
          const ImagePointsVecVec &imagePoints, const Size& image_size,
          vector< Vec3d > &rvecs, 
          vector< Vec3d > &tvecs,
          int flags = 0, 
          cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  ) { return -1; }



      virtual void projectPoints( const ObjectPointsVec &objectPoints, 
          const Vec3d &_rvec, const Vec3d &_tvec, ImagePointsVec &imagePoints, 
          cv::OutputArray jacobian = cv::noArray()) const = 0;

      //-- Undistortion functions --
      void undistortPoints( const ImagePointsVec &distorted, 
          ImagePointsVec &undistorted, 
          const Mat &R = cv::Mat::eye(3,3,CV_64F), 
          const Mat &P = cv::Mat());

      virtual void initUndistortRectifyMap( const Mat &R, const Mat &P,
          const cv::Size& size, int m1type, Mat &map1, Mat &map2 );

      void undistortImage( const Mat &distorted, Mat &undistorted,
          const Mat &Knew, const Size& new_size);

      static Matx33d InitialCameraEstimate( const Size &image_size )
      {
        float fEstimate = std::max( image_size.width, image_size.height )/ CV_PI;
        return Matx33d( fEstimate, 0, image_size.width/2.0 - 0.5,
            0, fEstimate, image_size.height/2.0 - 0.5,
            0, 0, 1. );
      }


    protected:
      virtual ImagePoint image( const ImagePoint &pt ) const = 0;
      virtual ImagePoint unimage( const ImagePoint &pt ) const = 0;

      virtual ImagePoint undistort( const ImagePoint &pw ) const = 0;
      virtual ImagePoint distort( const ObjectPoint &w ) const = 0;

      Camera() {;}
  };

  class PinholeCamera : public Camera {
    public:

      enum{
        CALIB_RECOMPUTE_EXTRINSIC   = (1<<0),
        CALIB_CHECK_COND            = (1<<1),
        CALIB_FIX_SKEW              = (1<<2),
        CALIB_FIX_INTRINSIC         = (1<<3)
      };
      static const int CALIB_FLAG_OFFSET = 4;


      PinholeCamera( void );
      PinholeCamera( const Matx33d &k );
      PinholeCamera( const Mat &k );

      virtual ~PinholeCamera() {;}

      virtual const std::string name( void ) const { return "pinhole"; }

      void setCamera( const Matx33d &k );
      void setCamera( double fx, double fy, double cx, double cy, double alpha = 1 );

      ImagePoint image( const ImagePoint &pt ) const;
      ImagePoint unimage( const ImagePoint &pt ) const;

      Matx33d matx( void ) const;
      Mat mat( void ) const;

      Vec2d  f( void ) const      { return Vec2d( _fx, _fy ); }
      double fx( void ) const     { return _fx; }
      double fy( void ) const     { return _fy; }
      Vec2d c( void ) const       { return Vec2d(_cx,_cy); }
      double cx( void ) const     { return _cx; }
      double cy( void ) const     { return _cy; }
      double alpha( void) const   { return _alpha; }


      virtual cv::FileStorage &write( cv::FileStorage &out ) const;

    protected:

      double _fx, _fy, _alpha, _cx, _cy;

  };

  class DistortionModel : public PinholeCamera {

    public:

      DistortionModel( void )
        : PinholeCamera() {;}

      DistortionModel( const Matx33d &cam )
        : PinholeCamera( cam ) {;}

      virtual ~DistortionModel() {;}

  };

}


#endif
