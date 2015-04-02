#ifndef __CALIBRATION_SERIALIZER_H__
#define __CALIBRATION_SERIALIZER_H__

#include <string>

#include "distortion_model.h"
#include "calibration_result.h"
#include "board.h"


namespace AplCam {

  using std::string;
  using Distortion::Camera;

  class CalibrationSerializer {
    public:

      CalibrationSerializer( void ) 
        : _camera( NULL ), _results( NULL ), _board( NULL )
      {;}

      CalibrationSerializer &setCamera( Camera *const cam )
      { _camera = cam; return *this; }

      CalibrationSerializer &setResult( CalibrationResult *const res )
      { _results = res; return *this; }

      CalibrationSerializer &setBoard( Board *const b )
      { _board = b; return *this; }

      string timeString( void ) const
      {
        time_t tt;
        time( &tt );
        struct tm *t2 = localtime( &tt );
        char buf[1024];
        strftime( buf, sizeof(buf)-1, "%c", t2 );
        return string(buf);
      }

      bool writeFile( const string &filename ) const
      {
        FileStorage fs( filename, FileStorage::WRITE );
        if( !fs.isOpened() ) return false;

        return serialize( fs );
      }

      bool serialize( string &str ) const
      {
        FileStorage fs("foo.yml", FileStorage::WRITE | FileStorage::MEMORY );
        bool res = serialize( fs );
        str = fs.releaseAndGetString();
        return res;
      }

      bool serialize( FileStorage &fs ) const
      {
        fs << "calibration_time" << timeString();

        if( _board ) {
          fs << "board_name" << _board->name;
          fs << "board_width" << _board->size().width;
          fs << "board_height" << _board->size().height;
          fs << "square_size" << _board->squareSize;
        }
        if( _camera) _camera->write( fs );
        if( _results ) _results->serialize( fs );

        return true;
      }

      //static void saveCameraParams( const string& filename,
      //    Size imageSize, const Board &board,
      //    const vector< Image > &imagesUsed,
      //    float aspectRatio, int flags,
      //    const DistortionModel *model, 
      //    const vector<Vec3d>& rvecs, const vector<Vec3d>& tvecs,
      //    const vector<float>& reprojErrs,
      //    const Distortion::ImagePointsVecVec &imagePoints,
      //    double totalAvgErr )
      //{
      //
      //  FileStorage out( filename, FileStorage::WRITE );
      //
      //  if( !rvecs.empty() || !reprojErrs.empty() )
      //    out << "nframes" << (int)std::max(rvecs.size(), reprojErrs.size());

      //  if( flags & CV_CALIB_FIX_ASPECT_RATIO )
      //    out << "aspectRatio" << aspectRatio;
      //
      //  if( flags != 0 )
      //  {
      //    sprintf( buf, "flags: %s%s%s%s",
      //        flags & CV_CALIB_USE_INTRINSIC_GUESS ? "+use_intrinsic_guess" : "",
      //        flags & CV_CALIB_FIX_ASPECT_RATIO ? "+fix_aspectRatio" : "",
      //        flags & CV_CALIB_FIX_PRINCIPAL_POINT ? "+fix_principal_point" : "",
      //        flags & CV_CALIB_ZERO_TANGENT_DIST ? "+zero_tangent_dist" : "" );
      //    cvWriteComment( *out, buf, 0 );
      //  }
      //
      //  out << "flags" << flags;
      //
      //
      //  out << "avg_reprojection_error" << totalAvgErr;
      //  if( !reprojErrs.empty() )
      //    out << "per_view_reprojection_errors" << Mat(reprojErrs);
      //
      //  //  if( !rvecs.empty() && !tvecs.empty() )
      //  //  {
      //  //    CV_Assert(rvecs[0].type() == tvecs[0].type());
      //  //    Mat bigmat((int)rvecs.size(), 6, rvecs[0].type());
      //  //    for( int i = 0; i < (int)rvecs.size(); i++ )
      //  //    {
      //  //      Mat r = bigmat(Range(i, i+1), Range(0,3));
      //  //      Mat t = bigmat(Range(i, i+1), Range(3,6));
      //  //
      //  //      CV_Assert(rvecs[i].rows == 3 && rvecs[i].cols == 1);
      //  //      CV_Assert(tvecs[i].rows == 3 && tvecs[i].cols == 1);
      //  //      //*.t() is MatExpr (not Mat) so we can use assignment operator
      //  //      r = rvecs[i].t();
      //  //      t = tvecs[i].t();
      //  //    }
      //  //    cvWriteComment( *out, "a set of 6-tuples (rotation vector + translation vector) for each view", 0 );
      //  //    out   << "extrinsic_parameters" << bigmat;
      //  //  }
      //
      //  //  out << "images_used" << "[";
      //  //  for( vector<Image>::const_iterator img = imagesUsed.begin(); img < imagesUsed.end(); ++img ) {
      //  //    out << img->fileName();
      //  //  }
      //  //  out << "]";
      //
      //  if( !imagePoints.empty() )
      //  {
      //    Mat imagePtMat((int)imagePoints.size(), (int)imagePoints[0].size(), CV_32FC2);
      //    for( size_t i = 0; i < imagePoints.size(); i++ )
      //    {
      //      Mat r = imagePtMat.row(i).reshape(2, imagePtMat.cols);
      //      Mat imgpti(imagePoints[i]);
      //      imgpti.copyTo(r);
      //    }
      //    out << "image_points" << imagePtMat;
      //  }
      //}


    protected:
      Camera *_camera;
      CalibrationResult *_results;
      Board *_board;

  };

}

#endif

