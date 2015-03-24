

#include "stereo_calibration.h"

using namespace cv;

namespace AplCam {

  void StereoCalibration::save( FileStorage &fs ) const
  {
    fs << "fundamental" << F;
    fs << "essential" << E;

    fs << "rotation" << R;
    fs << "translation" << t;


    //
    //  if( !rvecs.empty() || !reprojErrs.empty() )
    //    fs << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());
    //  fs << "image_width" << imageSize.width;
    //  fs << "image_height" << imageSize.height;
    //  fs << "board_name" << board.name;
    //  fs << "board_width" << board.size().width;
    //  fs << "board_height" << board.size().height;
    //  fs << "square_size" << board.squareSize;
    //
    //  if( flags & CV_CALIB_FIX_ASPECT_RATIO )
    //    fs << "aspectRatio" << aspectRatio;
    //
    //  if( flags != 0 )
    //  {
    //    sprintf( buf, "flags: %s%s%s%s",
    //        flags & CV_CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
    //        flags & CV_CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
    //        flags & CV_CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
    //        flags & CV_CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "" );
    //    cvWriteComment( *fs, buf, 0 );
    //  }
    //
    //  fs << "flags" << flags;
    //
    //  fs << "camera_matrix" << cameraMatrix;
    //  fs << "distortion_coefficients" << distCoeffs;
    //
    //  fs << "avg_reprojection_error" << totalAvgErr;
    //  if( !reprojErrs.empty() )
    //    fs << "per_view_reprojection_errors" << Mat(reprojErrs);
    //
    //  if( !rvecs.empty() && !tvecs.empty() )
    //  {
    //    CV_Assert(rvecs[0].type() == tvecs[0].type());
    //    Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
    //    for( int i = 0; i < (int)rvecs.size(); i++ )
    //    {
    //      Mat r = bigmat(Range(i, i+1), Range(0,3));
    //      Mat t = bigmat(Range(i, i+1), Range(3,6));
    //
    //      CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
    //      CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
    //      //*.t() is MatExpr (not Mat) so we can use assignment operator
    //      r = rvecs[i].t();
    //      t = tvecs[i].t();
    //    }
    //    cvWriteComment( *fs, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
    //    fs   << "extrinsic_parameters" << bigmat;
    //  }
    //
    //  fs << "images_used" << "[";
    //  for( vector<Image>::const_iterator img = imagesUsed.begin(); img < imagesUsed.end(); ++img ) {
    //    fs << img->fileName();
    //  }
    //  fs << "]";
    //
    //  if( !imagePoints.empty() )
    //  {
    //    Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
    //    for( int i = 0; i < (int)imagePoints.size(); i++ )
    //    {
    //      Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
    //      Mat imgpti(imagePoints[i]);
    //      imgpti.copyTo(r);
    //    }
    //    fs << "image_points" << imagePtMat;
    //  }
  }

}
