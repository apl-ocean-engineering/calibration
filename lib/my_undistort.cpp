
#include <gsl/gsl_vector.h>
#include <gsl/gsl_multiroots.h>

#include "my_undistort.h"

#include <iostream>

using namespace cv;
using namespace std;


struct UndistParams {
  UndistParams( double x, double y, double *distCoeffs )
    : xd( x ), yd( y ), k( distCoeffs ) {;}

  double xd, yd;
  double *k;
};

int distortionError( const gsl_vector *xy, void *params, gsl_vector *err )
{
  struct UndistParams *p = (struct UndistParams *)params;
  double *k = p->k;

  const double x = gsl_vector_get( xy, 0 );
  const double y = gsl_vector_get( xy, 1 );

  double r2 = x*x + y*y;
  double dist = (1 + ((k[4]*r2 + k[1])*r2 + k[0])*r2)/(1 + ((k[7]*r2 + k[6])*r2 + k[5])*r2);
  double deltaX = 2*k[2]*x*y + k[3]*(r2 + 2*x*x);
  double deltaY = k[2]*(r2 + 2*y*y) + 2*k[3]*x*y;

  // Distorted position given current x,y and distortion params
  double xest = dist * x + deltaX;
  double yest = dist * y + deltaY;

  gsl_vector_set( err, 0, xest - p->xd );
  gsl_vector_set( err, 1, yest - p->yd );

  return GSL_SUCCESS;
}


static void printState( size_t iter, gsl_multiroot_fsolver *s )
{
  cout << "Iter " << iter << " : (x,y) = " << gsl_vector_get( s->x, 0 ) << " " << gsl_vector_get( s->x, 1 ) << 
    " err(x,y) " << gsl_vector_get( s->f, 0 ) << " " << gsl_vector_get( s->f, 1 ) << endl;
}

void myUndistortPoints( InputArray _src, OutputArray _dst,
    InputArray _cameraMatrix,
    InputArray _distCoeffs,
    InputArray _Rmat,
    InputArray _Pmat )
{
  Mat src = _src.getMat(), cameraMatrix = _cameraMatrix.getMat();
  Mat distCoeffs = _distCoeffs.getMat(), R = _Rmat.getMat(), P = _Pmat.getMat();

  CV_Assert( src.isContinuous() && (src.depth() == CV_32F || src.depth() == CV_64F) &&
      ((src.rows == 1 && src.channels() == 2) || src.cols*src.channels() == 2));

  _dst.create(src.size(), src.type(), -1, true);
  Mat dst = _dst.getMat();

  //CvMat _csrc = src, _cdst = dst, _ccameraMatrix = cameraMatrix;
  //CvMat matR, matP, _cdistCoeffs, *pR=0, *pP=0, *pD=0;
  //if( R.data )
  //    pR = &(matR = R);
  //if( P.data )
  //    pP = &(matP = P);
  //if( distCoeffs.data )
  //    pD = &(_cdistCoeffs = distCoeffs);


  double A[3][3], k[8] = {0,0,0,0,0,0,0,0}, fx, fy, ifx, ify, cx, cy;
  Mat matK( distCoeffs.size(), CV_64F, k ),
      matA( 3, 3, CV_64F, A );

  //CvMat _RR=cvMat(3, 3, CV_64F, RR);
  //const CvPoint2D32f* srcf;
  //const CvPoint2D64f* srcd;
  //CvPoint2D32f* dstf;
  //CvPoint2D64f* dstd;
  //int stype, dtype;
  //int sstep, dstep;
  //int i, j, n, iters = 1;

  int iters = 1;

  //    CV_Assert( CV_IS_MAT(src) && CV_IS_MAT(dst) &&
  //        (src->rows == 1 || src->cols == 1) &&
  //        (dst->rows == 1 || dst->cols == 1) &&
  //        src->cols + src->rows - 1 == dst->rows + dst->cols - 1 &&
  //        (CV_MAT_TYPE(src->type) == CV_32FC2 || CV_MAT_TYPE(src->type) == CV_64FC2) &&
  //        (CV_MAT_TYPE(dst->type) == CV_32FC2 || CV_MAT_TYPE(dst->type) == CV_64FC2));

  CV_Assert( cameraMatrix.size().width == 3 && 
      cameraMatrix.size().height == 3 );
  cameraMatrix.convertTo( matA, CV_64F ); 

  if( !distCoeffs.empty() )
  {
    //CV_Assert( CV_IS_MAT(distCoeffs) &&
    //    (distCoeffs->rows == 1 || distCoeffs->cols == 1) &&
    //    (distCoeffs->rows*distCoeffs->cols == 4 ||
    //     distCoeffs->rows*distCoeffs->cols == 5 ||
    //     distCoeffs->rows*distCoeffs->cols == 8));

    distCoeffs.convertTo( matK, CV_64F );
    iters = 5;
  }

  double RR[3][3];
  Mat matR( 3, 3, CV_64F, RR );
  matR = Mat::eye(3,3,CV_64F);

  if( !R.empty() ) {
    CV_Assert( R.size().width*R.size().height == 9 );
    R.convertTo( matR, CV_64F );
  } 


  if( !P.empty() ) {
    Mat matP( 3,3, CV_64F );
    P.convertTo( matP, CV_64F );

    matR = matP * matR; 
  }

  //    {
  //        double PP[3][3];
  //        CvMat _P3x3, _PP=cvMat(3, 3, CV_64F, PP);
  //        CV_Assert( CV_IS_MAT(matP) && matP->rows == 3 && (matP->cols == 3 || matP->cols == 4));
  //        cvConvert( cvGetCols(matP, &_P3x3, 0, 3), &_PP );
  //        cvMatMul( &_PP, &_RR, &_RR );
  //    }

  //    srcf = (const CvPoint2D32f*)_src->data.ptr;
  //    srcd = (const CvPoint2D64f*)_src->data.ptr;
  //    dstf = (CvPoint2D32f*)_dst->data.ptr;
  //    dstd = (CvPoint2D64f*)_dst->data.ptr;
  int stype = src.depth();
  int dtype = dst.depth();
  //    sstep = _src->rows == 1 ? 1 : _src->step/CV_ELEM_SIZE(stype);
  //    dstep = _dst->rows == 1 ? 1 : _dst->step/CV_ELEM_SIZE(dtype);

  // This will blow up horribly if this isn't true...
  assert( src.isContinuous() && dst.isContinuous() );

  int n = src.size().height * src.size().width;

  fx = A[0][0];
  fy = A[1][1];
  ifx = 1./fx;
  ify = 1./fy;
  cx = A[0][2];
  cy = A[1][2];

  float *sptrf = src.ptr<float>(0);
  double *sptrd = src.ptr<double>(0);
  float *dptrf = dst.ptr<float>(0);
  double *dptrd = dst.ptr<double>(0);

  const gsl_multiroot_fsolver_type *T;
  gsl_multiroot_fsolver *s;
  T = gsl_multiroot_fsolver_dnewton;
  s = gsl_multiroot_fsolver_alloc (T, 2);

  const int maxIters = 1000;
  gsl_vector *states = gsl_vector_alloc(2);

  int idx = 0;
  for( int i = 0; i < n; i++, idx+=2 )
  {
    double x, y, x0, y0;
    if( stype == CV_32F )
    {
      x = sptrf[idx];
      y = sptrf[idx+1];
    }
    else
    {
      x = sptrd[idx];
      y = sptrd[idx+1];
    }

    x0 = (x - cx)*ifx;
    y0 = (y - cy)*ify;

    // Outsource solution to GSL
    gsl_vector_set( states, 0, x0 );
    gsl_vector_set( states, 1, y0 );

    UndistParams params( x0, y0, k );

    gsl_multiroot_function f = {&distortionError, 2, &params};
    gsl_multiroot_fsolver_set (s, &f, states);

    int iter = 0, status;
    do {
      iter++;

      status = gsl_multiroot_fsolver_iterate(s);

      printState( iter, s );
      if( status ) break;

      status = gsl_multiroot_test_residual( s->f, 1e-7 );
    } while( status ==GSL_CONTINUE && iter < maxIters );


    x = gsl_vector_get( s->x, 0 );
    y = gsl_vector_get( s->x, 1 );

    double xx = RR[0][0]*x + RR[0][1]*y + RR[0][2];
    double yy = RR[1][0]*x + RR[1][1]*y + RR[1][2];
    double ww = 1./(RR[2][0]*x + RR[2][1]*y + RR[2][2]);
    x = xx*ww;
    y = yy*ww;

    if( dtype == CV_32F )
    {
      dptrf[idx] = (float)x;
      dptrf[idx+1] = (float)y;
    }
    else
    {
      dptrd[idx] = (float)x;
      dptrd[idx+1] = (float)y;
    }
  }

  gsl_multiroot_fsolver_free (s);
  gsl_vector_free( states );

}


