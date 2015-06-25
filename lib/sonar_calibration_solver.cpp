

#include <ceres/ceres.h>
#include <ceres/rotation.h>

#include <iostream>

#include "sonar_calibration_solver.h"

using namespace std;



void SonarCalibrationSolver::solve( const SonarCalData &data )
{

}


#ifdef false

  struct ReprojectionError {
    ReprojectionError(double obs_x, double obs_y, double world_x, double world_y )
      : observedX(obs_x), observedY(obs_y), worldX( world_x ), worldY( world_y ) {;}

    template <typename T>
      void txToCameraFrame( const T* const pose, T *p ) const
      {
        const T point[3] = { T( worldX ), T( worldY ), T( 0.0 ) };
        ceres::AngleAxisRotatePoint(pose, point, p);
        p[0] += pose[3]; 
        p[1] += pose[4]; 
        p[2] += pose[5];
      }

    template <typename T>
      bool projectAndComputeError(const T* const camera,
          const T* const alpha,
          const T* const pp,
          T* residuals) const
      {
        const T &fx = camera[0];
        const T &fy = camera[1];
        const T &cx = camera[2];
        const T &cy = camera[3];
        const T &xpp = pp[0],
              &ypp = pp[1];

        T predictedX = fx*(xpp + alpha[0]*ypp) + cx;
        T predictedY = fy* ypp                 + cy;

        // The error is the difference between the predicted and observed position.
        residuals[0] = predictedX - T(observedX);
        residuals[1] = predictedY - T(observedY);
        return true;
      }


    double observedX, observedY;
    double worldX, worldY;
  };

  struct RadialDistortionReprojError : public ReprojectionError {
    RadialDistortionReprojError(double obs_x, double obs_y, double world_x, double world_y )
      : ReprojectionError( obs_x, obs_y, world_x, world_y ) 
    {;}


    template <typename T>
      bool operator()(const T* const camera,
          const T* const alpha,
          const T* const k12,
          const T* const p12,
          const T* const k3,
          const T* const k456,
          const T* const pose, 
          T* residuals) const
      {
        //
        // camera is a 4-vector
        //    2 focal length
        //    2 camera center
        //
        // alpha is a 1-vector (separate so it can be set fixed/constant)
        //
        // k12 is a 2-vector: k1 and k2 by opencv
        // p12 is a 2-vector: p1 and p2 by opencv
        // k3  is a 1-vector: k3 by openv
        // k456 is a 3-vector: k456 by opencv
        //
        // pose is a 6-vector
        //    3 angles
        //    3 translations
        // pose[0,1,2] are an angle-axis rotation.
        //
        T p[3];
        txToCameraFrame( pose, p );

        const T &k1 = k12[0], &k2 = k12[1];
        const T &p1 = p12[0], &p2 = p12[1];
        const T &k4 = k456[0], &k5 = k456[1], &k6 = k456[2];

        T xp = p[0]/p[2], yp = p[1]/p[2];
        T r2 = xp*xp + yp*yp;
        T r4 = r2*r2;
        T r6 = r2*r4;

        T pp[2];
        pp[0] = xp * ( T(1) + k1*r2 + k2*r4 + k3[0]*r6 ) / ( T(1) + k4*r2 + k5*r4 + k6*r6 ) + T(2)*p1*xp*yp + p2*(r2 + T(2)*xp*xp);
        pp[1] = yp * ( T(1) + k1*r2 + k2*r4 + k3[0]*r6 ) / ( T(1) + k4*r2 + k5*r4 + k6*r6 ) + p1*(r2 + T(2)*yp*yp) + T(2)*p2*xp*yp;

        return projectAndComputeError( camera, alpha, pp, residuals );
      }

  };

  struct RadialDistortionFactory {
    RadialDistortionFactory( double *camera, double *alpha, double *dist, ceres::LossFunction *lossF = NULL )
      : camera_(camera), alpha_(alpha), dist_(dist), lossFunc_( lossF )
    {;}

    void add( ceres::Problem &problem, const ObjectPoint &obj, const ImagePoint &img, double *pose )
    {
      ceres::CostFunction *costFunction = (new ceres::AutoDiffCostFunction<RadialDistortionReprojError, 2, 4,1,2,2,1,3, 6>(
            new RadialDistortionReprojError( img[0], img[1], obj[0], obj[1] ) ) );

      double *k12  = &(dist_[0]);
      double *p12  = &(dist_[2]);
      double *k3   = &(dist_[4]);
      double *k456 = &(dist_[5]);

      problem.AddResidualBlock( costFunction, lossFunc_, camera_, alpha_, k12, p12, k3, k456, pose );
    }

    double *camera_, *alpha_, *dist_, *pose_;
    ceres::LossFunction *lossFunc_;
  };

  bool CeresRadialPolynomial::doCalibrate(
      const ObjectPointsVecVec &objectPoints, 
      const ImagePointsVecVec &imagePoints, 
      const Size& image_size,
      CalibrationResult &result,
      int flags, 
      cv::TermCriteria criteria)
  {

    // Check and see if the camera matrix has been initialized
    if( norm( matx(), Mat::eye(3,3,CV_64F) ) < 1e-9 )
      setCamera( Camera::InitialCameraEstimate( image_size ) );

    int totalPoints = 0;
    int goodImages = 0;

    for( size_t i = 0; i < objectPoints.size(); ++i )  {
      if( result.status[i] ) {
        ImagePointsVec undistorted =  unwarp( normalize( imagePoints[i] ) );

        // Found the approach provided by initExtrinsics to be more reliable (!)
        // will need to investigate why that is.
        bool pnpRes = solvePnP( objectPoints[i], undistorted, Mat::eye(3,3,CV_64F), Mat(), 
            result.rvecs[i], result.tvecs[i], false, CV_ITERATIVE );

        //cout << "Pnp: " << (pnpRes ? "" : "FAIL") << endl << result.rvecs[i] << endl << result.tvecs[i] << endl;
        //initExtrinsics( imagePoints[i], objectPoints[i], rvecs[i], tvecs[i] );
        //cout << "initExtrinsics: " << endl << rvecs[i] << endl << tvecs[i] << endl;

        if( !pnpRes ) { 
          result.status[i] = false; 
          continue; 
        }

        ++goodImages;
        totalPoints += objectPoints[i].size();
      }
    }

    cout << "From " << objectPoints.size() << " images, using " << totalPoints << " from " << goodImages << " images" << endl;

    double camera[4] = { _fx, _fy, _cx, _cy };
    double alpha = _alpha;

    // Inherently fragile.  Store coeffs in a double array instead?
    //    double k12[2] = { _distCoeffs[0], _distCoeffs[1] },
    //           p12[2] = { _distCoeffs[2], _distCoeffs[3] },
    //           k3[1]  = { _distCoeffs[4] },
    //           k456[3] = { _distCoeffs[5], _distCoeffs[6], _distCoeffs[7] };

    if( ! (flags & CV_CALIB_RATIONAL_MODEL ) ) {
      _distCoeffs[5] = 0.0;
      _distCoeffs[6] = 0.0;
      _distCoeffs[7] = 0.0;
    } else {
      cout << "Using rational model (with k4-k6)" << endl;
    }

    if( flags & CV_CALIB_ZERO_TANGENT_DIST ) {
      _distCoeffs[2] = 0.0;
      _distCoeffs[3] = 0.0;
      cout << "Fixing tangential distortion to zero" << endl;
    }

    ceres::LossFunction *lossFunc =  NULL;
    if( flags & CALIB_HUBER_LOSS ) {
      LOG(INFO) << "Using Huber loss function";

      // Need to set parameter, which is in the units
      // of the residual (pixels, in this case)
      // It is squared internally
      lossFunc = new ceres::HuberLoss( 4.0 );
    }


    double *pose = new double[ goodImages * 6];
    RadialDistortionFactory factory(  camera, &alpha, (_distCoeffs.val), lossFunc );

    ceres::Problem problem;
    for( size_t i = 0, idx = 0; i < objectPoints.size(); ++i ) {
      if( result.status[i] ) {

        double *p = &( pose[idx*6] );

        // Mildly awkward
        p[0] = result.rvecs[i][0];
        p[1] = result.rvecs[i][1];
        p[2] = result.rvecs[i][2];
        p[3] = result.tvecs[i][0];
        p[4] = result.tvecs[i][1];
        p[5] = result.tvecs[i][2];

        ++idx;

        for( size_t j = 0; j < imagePoints[i].size(); ++j ) {
          factory.add( problem, objectPoints[i][j], imagePoints[i][j], p );
        }
      }
    }

    //if( flags & CALIB_FIX_SKEW )
    problem.SetParameterBlockConstant( &alpha );

    // Fragile
    if( flags & CV_CALIB_ZERO_TANGENT_DIST ) problem.SetParameterBlockConstant( &(_distCoeffs[2]) );
    if( ! (flags & CV_CALIB_RATIONAL_MODEL ) ) problem.SetParameterBlockConstant( &(_distCoeffs[5]) );

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations = criteria.maxCount;
    options.minimizer_progress_to_stdout = true;

    // This should be configurable by the end user
    options.num_threads = boost::thread::hardware_concurrency();

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    for( size_t i = 0, idx=0; i < objectPoints.size(); ++i ) {
      if( result.status[i] ) {
        result.rvecs[i] = Vec3d( &(pose[idx*6]) );
        result.tvecs[i] = Vec3d( &(pose[idx*6+3]) );
        ++idx;
      }
    }

    delete[] pose;

    result.totalTime = summary.total_time_in_seconds;

    // N.b. the Ceres cost is 1/2 || f(x) ||^2
    //
    // Whereas we usually use the RMS reprojection error
    //
    // sqrt( 1/N  ||f(x)||^2
    //
    // So rms = sqrt( 2/N final_cost )
    //
    result.residual = summary.final_cost;
    result.numPoints = totalPoints;
    result.numImages = goodImages;
    result.good = summary.IsSolutionUsable();

    set(camera, alpha);

    result.rms = reprojectionError( objectPoints, result.rvecs, result.tvecs, imagePoints, result.reprojErrors, result.status );

    cout << "Final camera: " << endl << matx() << endl;
    cout << "Final distortions: " << endl << _distCoeffs << endl;

    return true;
  }


#endif
