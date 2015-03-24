
#ifndef __DISTORTION_ANGULAR_POLYNOMIAL_H__
#define __DISTORTION_ANGULAR_POLYNOMIAL_H__

#include "distortion_model.h"

namespace Distortion {

  using cv::Vec4d;

  class AngularPolynomial : public DistortionModel {
    public:

      static const Vec4d ZeroDistortion;

      AngularPolynomial( void );
      AngularPolynomial( const Vec4d &distCoeffs );
      AngularPolynomial( const Vec4d &distCoeffs, const Matx33d &cam );

      void set(const cv::Vec2d& f, const cv::Vec2d& c, const double &alpha = 0, const cv::Vec4d& k = ZeroDistortion )
      {
        setCamera( f[0], f[1], c[0], c[1], alpha );
        _distCoeffs = k;
      }

      static const std::string Name( void ) { return "AngularPolynomial"; }
      virtual const std::string name( void ) const { return AngularPolynomial::Name(); }


      //      void set( const AngularPolynomial &other )
      //      { set( other.f(), other.c(), other.alpha(), other.distCoeffs() ); }

      void set( const double *c, const double alpha )
      {
        setCamera( c[0], c[1], c[2], c[3], alpha );
        _distCoeffs = Vec4d( &c[4] ); // c[4], c[5], c[6], c[8] );
      }

      Vec4d distCoeffs( void ) const    { return _distCoeffs; }

      static AngularPolynomial Calibrate( const ObjectPointsVecVec &objectPoints, 
          const ImagePointsVecVec &imagePoints, const Size& image_size,
          vector< Vec3d > &rvecs, 
          vector< Vec3d > &tvecs,
          int flags = 0, 
          cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  );

      virtual double calibrate( const ObjectPointsVecVec &objectPoints, 
          const ImagePointsVecVec &imagePoints, const Size& image_size,
          vector< Vec3d > &rvecs, 
          vector< Vec3d > &tvecs,
          int flags = 0, 
          cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, DBL_EPSILON)  );

      virtual cv::FileStorage &write( cv::FileStorage &out ) const;
      static AngularPolynomial *Load( cv::FileStorage &in );


      virtual ImagePoint unwarp( const ImagePoint &pw ) const;
      virtual ImagePointsVec unwarp( const ImagePointsVec &pw ) const;

      virtual ImagePoint warp( const ObjectPoint &w ) const;


    protected: 

      static Matx33d InitialCameraEstimate( const Size &image_size );

      //      void calibrateExtrinsics( const ObjectPointsVecVec &objectPoints,
      //          const ImagePointsVecVec &imagePoints,
      //          const int check_cond,
      //          const double thresh_cond,
      //          vector< Vec3d > &omc, 
      //          vector< Vec3d > &Tc );

      //      void initExtrinsics(const ImagePointsVec& _imagePoints, const ObjectPointsVec& _objectPoints, 
      //          Vec3d& omckk, Vec3d& Tckk);
      //
      //      Mat computeHomography(Mat m, Mat M);
      //
      //void normalizePixels(const ImagePointsVec& imagePoints, Mat &normalized);

      //void computeExtrinsicRefine(const ImagePointsVec& imagePoints, const ObjectPointsVec& objectPoints, 
      //    Mat &rvec, Mat &tvec, 
      //    Mat &J, const int MaxIter,
      //    const double thresh_cond);

      // Internal functions for projection points given an IntrinsicParams
      //void projectPoints(const ObjectPointsVec &objectPoints, ImagePointsVec &imagePoints,
      //    const Vec3d &_rvec, const Vec3d &_tvec,
      //    cv::OutputArray jacobian);

      // This was formerly in its own anonymous namespace.
      // Maybe it should still be there...
      //void subMatrix(const Mat& src, Mat& dst, const vector<int>& cols, const vector<int>& rows);

      cv::Vec4d _distCoeffs;

  };

}


#endif
