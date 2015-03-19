//
// Refactoring the OpenCV distortion model, starting with the RadialPolynomial model.
// Based on a heavily hacked version of fisheye.cpp from OpenCV.
//
/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                           License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000-2008, Intel Corporation, all rights reserved.
// Copyright (C) 2009-2011, Willow Garage Inc., all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of the copyright holders may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#include <math.h>

#include "distortion_radial_polynomial.h"

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <iostream>
using namespace std;


static std::ostream& operator <<(std::ostream& stream, const Distortion::RadialPolynomial &p )
{
  stream << p.f()[0] << " " << p.f()[1] << ", " << p.c()[0] << " " << p.c()[1] << ", " << p.alpha() << ", " << p.distCoeffs()[0] << " " << p.distCoeffs()[1] << " " << p.distCoeffs()[2] << " " << p.distCoeffs()[3];
  return stream;
}

namespace Distortion {

  using namespace cv;
  using namespace std;

  RadialPolynomial::RadialPolynomial( void )
    : DistortionModel(), _distCoeffs( 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 )
  {;}

  RadialPolynomial::RadialPolynomial( const Vec4d &d )
    : DistortionModel(), _distCoeffs( d[0], d[1], d[2], d[3], 0., 0., 0., 0. )
  {;}

  RadialPolynomial::RadialPolynomial( const Vec5d &d )
    : DistortionModel(), _distCoeffs( d[0], d[1], d[2], d[3], d[4], 0., 0., 0. )
  {;}

  RadialPolynomial::RadialPolynomial( const Vec8d &distCoeffs )
    : DistortionModel(), _distCoeffs( distCoeffs )
  {;}

  RadialPolynomial::RadialPolynomial( const Vec4d &d, const Matx33d &cam )
    : DistortionModel( cam ), _distCoeffs( d[0], d[1], d[2], d[3], 0., 0., 0., 0. )
  {;}

  RadialPolynomial::RadialPolynomial( const Vec5d &d, const Matx33d &cam )
    : DistortionModel( cam ), _distCoeffs( d[0], d[1], d[2], d[3], d[4], 0., 0., 0. )
  {;}

  RadialPolynomial::RadialPolynomial( const Vec8d &distCoeffs, const Matx33d &cam )
    : DistortionModel( cam ), _distCoeffs( distCoeffs )
  {;}


  // Static version uses a reasonable estimate based on image size
  RadialPolynomial RadialPolynomial::Calibrate( 
      const ObjectPointsVecVec &objectPoints, 
      const ImagePointsVecVec &imagePoints, 
      const Size& image_size,
      vector< Vec3d > &rvecs, 
      vector< Vec3d > &tvecs,
      int flags, 
      cv::TermCriteria criteria)
  {
    RadialPolynomial fe( InitialDistortionEstimate(), InitialCameraEstimate( image_size ) );
    fe.calibrate( Mat(objectPoints), Mat(imagePoints), image_size, rvecs, tvecs, flags, criteria );
    return fe;
  }


  // Ceres functor for solving calibration problem
  //  Based on the Bundler solver used in their examples
//  struct CalibReprojectionError {
//    CalibReprojectionError(double observed_x, double observed_y, double world_x, double world_y )
//      : observed_x(observed_x), observed_y(observed_y), worldX( world_x ), worldY( world_y ) {}
//
//    template <typename T>
//      bool operator()(const T* const camera,
//          const T* const alpha,
//          const T* const pose, 
//          T* residuals) const
//      {
//        // pose is a 6-vector
//        //    3 angles
//        //    3 translations
//        //
//        // alpha is a 1-vector (separate so it can be set Constant/Variable)
//        //
//        // camera i s 8-vector
//        //    2 focal length
//        //    2 camera center
//        //    4 distortion params
//        //
//        //
//        // camera[0,1,2] are the angle-axis rotation.
//        T point[3] = { T( worldX ), 
//          T( worldY ), 
//          T( 0.0 ) };
//        T p[3];
//        ceres::AngleAxisRotatePoint(pose, point, p);
//        p[0] += pose[3]; p[1] += pose[4]; p[2] += pose[5];
//
//        T theta = atan2( sqrt( p[0]*p[0] + p[1]*p[1] ), p[2]  );
//        T psi   = atan2( p[1], p[0] );
//
//        const T &fx = camera[0];
//        const T &fy = camera[1];
//        const T &cx = camera[2];
//        const T &cy = camera[3];
//        const T &k1 = camera[4];
//        const T &k2 = camera[5];
//        const T &k3 = camera[6];
//        const T &k4 = camera[7];
//
//        T theta2 =  theta*theta;
//        T theta4 =  theta2*theta2;
//        T theta6 = theta4*theta2;
//        T theta8 = theta4*theta4;
//
//        T thetaDist = theta * ( T(1) + k1*theta2 + k2 *theta4 + k3*theta6 + k4*theta8);
//
//        T xdn = thetaDist * cos( psi ),
//          ydn = thetaDist * sin( psi );
//
//        T predictedX = fx*(xdn + alpha[0]*ydn) + cx;
//        T predictedY = fy* ydn              + cy;
//
//        // The error is the difference between the predicted and observed position.
//        residuals[0] = predictedX - T(observed_x);
//        residuals[1] = predictedY - T(observed_y);
//        return true;
//      }

//    // Factory to hide the construction of the CostFunction object from
//    // the client code.
//    static ceres::CostFunction* Create(const double observed_x, const double observed_y, 
//        const double world_x, const double world_y ) {
//      return (new ceres::AutoDiffCostFunction<CalibReprojectionError, 2, 8, 1, 6>(
//            new CalibReprojectionError(observed_x, observed_y, world_x, world_y)));
//    }
//
//    double observed_x;
//    double observed_y;
//
//    double worldX, worldY;
//  };

  double RadialPolynomial::calibrate(
      const ObjectPointsVecVec &objectPoints, 
      const ImagePointsVecVec &imagePoints, 
      const Size& imageSize,
      vector< Vec3d > &rvecs, 
      vector< Vec3d > &tvecs,
      int flags, 
      cv::TermCriteria criteria)
  {

    Mat camera( mat() );
    Mat dist( _distCoeffs );

    double rms = calibrateCamera( objectPoints, imagePoints, imageSize, camera, dist, rvecs, tvecs, flags, criteria );

    setCamera( camera );
    dist.copyTo( _distCoeffs, CV_64F );

    return rms;
  }

  void RadialPolynomial::projectPoints( const ObjectPointsVec &objectPoints, 
      const Vec3d &rvec, const Vec3d &tvec, ImagePointsVec &imagePoints, 
      OutputArray jacobian) const
  {
    
    cv::projectPoints( objectPoints, rvec, tvec, mat(), Mat( _distCoeffs ), imagePoints, jacobian );
  }


  ImagePoint RadialPolynomial::distort( const ObjectPoint &w ) const
  {
    double theta = atan2( sqrt( w[0]*w[0] + w[1]*w[1] ), w[2] );
    double psi = atan2( w[1], w[0] );

    double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta4*theta4;
    double theta_d = theta * (1 + _distCoeffs[0]*theta2 + _distCoeffs[1]*theta4 + _distCoeffs[2]*theta6 + _distCoeffs[3]*theta8);

    return Vec2d( theta_d*cos( psi ), theta_d*sin(psi) );
  }

  ImagePoint RadialPolynomial::undistort( const ImagePoint &pw ) const
  {
    double scale = 1.0;

    double theta_d = sqrt(pw[0]*pw[0] + pw[1]*pw[1]);
    if (theta_d > 1e-8)
    {
      // compensate distortion iteratively
      double theta = theta_d;
      for(int j = 0; j < 10; j++ )
      {
        double theta2 = theta*theta, theta4 = theta2*theta2, theta6 = theta4*theta2, theta8 = theta6*theta2;
        theta = theta_d / (1 + _distCoeffs[0] * theta2 + _distCoeffs[1] * theta4 + _distCoeffs[2] * theta6 + _distCoeffs[3] * theta8);
      }

      scale = std::tan(theta) / theta_d;
    }

    Vec2d pu = pw * scale; //undistorted point

    return pu;
  }

  FileStorage &RadialPolynomial::write( FileStorage &out ) const
  {
    DistortionModel::write( out );
    out << "distortion_coefficients" << _distCoeffs;

    return out;
  }

  RadialPolynomial *RadialPolynomial::Load( cv::FileStorage &in )
  {
    Mat kmat, distmat;

    in["camera_matrix"] >> kmat;
    in["distortion_coefficients"] >> distmat;

    Matx33d k;
    Vec8d dist;

    kmat.copyTo( k, CV_64F );
    distmat.copyTo( dist, CV_64F );

    return new RadialPolynomial( dist, k );

  }
}


