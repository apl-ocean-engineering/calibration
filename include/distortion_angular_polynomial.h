
#ifndef __DISTORTION_ANGULAR_POLYNOMIAL_H__
#define __DISTORTION_ANGULAR_POLYNOMIAL_H__

#include "distortion_model.h"

namespace Distortion {

  class AngularPolynimalEstimable;

  class AngularPolynomial : public DistortionModel {
    public:

      enum{
        CALIB_RECOMPUTE_EXTRINSIC   = (1<<1),
        CALIB_CHECK_COND            = (1<<2),
        CALIB_FIX_SKEW              = (1<<3),
        CALIB_FIX_K1                = (1<<4),
        CALIB_FIX_K2                = (1<<5),
        CALIB_FIX_K3                = (1<<6),
        CALIB_FIX_K4                = (1<<7),
        CALIB_FIX_INTRINSIC         = (1<<8)
      };


      AngularPolynomial( void );
      AngularPolynomial( const Vec4d &distCoeffs );
      AngularPolynomial( const Vec4d &distCoeffs, const Matx33d &cam );

      void set(const cv::Vec2d& f, const cv::Vec2d& c, const double &alpha = 0, const cv::Vec4d& k = Vec4d(0,0,0,0) )
      {
        setCamera( f[0], f[1], c[0], c[1], alpha );
        _distCoeffs = k;
      }

      void set( const AngularPolynomial &other )
      { set( other.f(), other.c(), other.alpha(), other.distCoeffs() ); }

      void set( const double *c )
      {
        setCamera( c[0], c[1], c[2], c[3], c[4] );
        _distCoeffs = Vec4d( c[5], c[6], c[7], c[8] );
      }


      Vec4d distCoeffs( void ) const    { return _distCoeffs; }

      static AngularPolynomial Calibrate( const ObjectPointsVecVec &objectPoints, 
          const ImagePointsVecVec &imagePoints, const Size& image_size,
          vector< Vec3d > &rvecs, 
          vector< Vec3d > &tvecs,
          int flags = 0, 
          cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  );

      double calibrate( const ObjectPointsVecVec &objectPoints, 
          const ImagePointsVecVec &imagePoints, const Size& image_size,
          vector< Vec3d > &rvecs, 
          vector< Vec3d > &tvecs,
          int flags = 0, 
          cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  );



      void undistortPoints( const vector< Point2d > &distorted, 
          vector< Point2d > &undistorted, 
          const Mat &R = cv::Mat::eye(3,3,CV_64F), 
          const Mat &P = cv::Mat());

      void projectPoints( const ObjectPointsVec &objectPoints, ImagePointsVec &imagePoints, 
          const cv::Affine3d& affine,
          cv::OutputArray jacobian = cv::noArray()) const;

      virtual void projectPoints( const ObjectPointsVec &objectPoints, ImagePointsVec &imagePoints, 
          const Vec3d &_rvec, const Vec3d &_tvec, 
          cv::OutputArray jacobian = cv::noArray() ) const;


    protected: 

      static Matx33d InitialCameraEstimate( const Size &image_size );

      void calibrateExtrinsics( const ObjectPointsVecVec &objectPoints,
          const ImagePointsVecVec &imagePoints,
          const int check_cond,
          const double thresh_cond,
          vector< Vec3d > &omc, 
          vector< Vec3d > &Tc );

      void initExtrinsics(const ImagePointsVec& _imagePoints, const ObjectPointsVec& _objectPoints, 
          Mat& omckk, Mat& Tckk);

      Mat computeHomography(Mat m, Mat M);

      void normalizePixels(const ImagePointsVec& imagePoints, Mat &normalized);

      void computeExtrinsicRefine(const ImagePointsVec& imagePoints, const ObjectPointsVec& objectPoints, 
          Mat &rvec, Mat &tvec, 
          Mat &J, const int MaxIter,
          const double thresh_cond);

      // Internal functions for projection points given an IntrinsicParams
      //void projectPoints(const ObjectPointsVec &objectPoints, ImagePointsVec &imagePoints,
      //    const Vec3d &_rvec, const Vec3d &_tvec,
      //    cv::OutputArray jacobian);

      // This was formerly in its own anonymous namespace.
      // Maybe it should still be there...
      void subMatrix(const Mat& src, Mat& dst, const vector<int>& cols, const vector<int>& rows);

      cv::Vec4d _distCoeffs;

  };

  class AngularPolynomialEstimable : public AngularPolynomial {
    public:

      AngularPolynomialEstimable() : AngularPolynomial(), isEstimate(9,0) {;}

      AngularPolynomialEstimable( const AngularPolynomial &other )
        : AngularPolynomial( other.distCoeffs(), other.matx() ), isEstimate(9,0) {;}

      AngularPolynomialEstimable( const Vec4d &distCoeffs, const Matx33d &cam )
        : AngularPolynomial( distCoeffs, cam ), isEstimate(9,0) {;}


      vector<int> isEstimate;

      AngularPolynomialEstimable operator+(const Mat& a);
      AngularPolynomialEstimable& operator=(const Mat& a);

      Vec4d normVec( void ) const;
      double deltaFrom( const AngularPolynomialEstimable &other ) const;

      friend class AngularPolynomial;

    protected:
      void computeJacobians( const ObjectPointsVecVec &objectPoints, 
          const ImagePointsVecVec &imagePoints,
          const vector< Vec3d > &omc, const vector< Vec3d > &Tc,
          const int check_cond, const double thresh_cond, 
          Mat& JJ2_inv, Mat& ex3);

      double estimateUncertainties( const ObjectPointsVecVec &objectPoints, 
          const ImagePointsVecVec &imagePoints,
          const vector< Vec3d > &omc, 
          const vector< Vec3d > &Tc,
          AngularPolynomialEstimable &errors, 
          Vec2d& std_err, 
          double thresh_cond, int check_cond );



  };

}


#endif
