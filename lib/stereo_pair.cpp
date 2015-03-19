
#include <opencv2/calib3d/calib3d.hpp>
#include <vector>

#include "stereo_pair.h"

namespace Distortion {

  typedef cv::Vec<double,8> Vec8d;

  using namespace cv;
  using namespace std;

  double stereoCalibrate( ObjectPointsVecVec _objectPoints,
      ImagePointsVecVec _imagePoints1,
      ImagePointsVecVec _imagePoints2,
      const PinholeCamera &cam1, const PinholeCamera &cam2,
      Size imageSize, OutputArray _Rmat, OutputArray _Tmat,
      OutputArray _Emat, OutputArray _Fmat, TermCriteria criteria,
      int flags )
  {

    // Before I get too involvved, what if I just un-distort all image points
    // then call the built-in function?

    ImagePointsVecVec _undistorted1, _undistorted2;

    std::transform( _imagePoints1.begin(), _imagePoints1.end(), 
        _undistorted1.begin(), cam1.makeVecUndistorter( ) );
    std::transform( _imagePoints2.begin(), _imagePoints2.end(), 
        _undistorted2.begin(), cam2.makeVecUndistorter( ) );

    return cv::stereoCalibrate( _objectPoints, _undistorted1, _undistorted2,
        cam1.mat(), cam2.mat(), Vec8d(), Vec8d(),
        imageSize, _Rmat, _Tmat, _Emat, _Fmat, criteria, flags );

  }
}

  //
  ////int rtype = CV_64F;
  //    Mat cameraMatrix1 = _cameraMatrix1.getMat();
  //    Mat cameraMatrix2 = _cameraMatrix2.getMat();
  //    Mat distCoeffs1 = _distCoeffs1.getMat();
  //    Mat distCoeffs2 = _distCoeffs2.getMat();
  //    cameraMatrix1 = prepareCameraMatrix(cameraMatrix1, rtype);
  //    cameraMatrix2 = prepareCameraMatrix(cameraMatrix2, rtype);
  //    distCoeffs1 = prepareDistCoeffs(distCoeffs1, rtype);
  //    distCoeffs2 = prepareDistCoeffs(distCoeffs2, rtype);
  //
  //    if( !(flags & CALIB_RATIONAL_MODEL) )
  //    {
  //        distCoeffs1 = distCoeffs1.rows == 1 ? distCoeffs1.colRange(0, 5) : distCoeffs1.rowRange(0, 5);
  //        distCoeffs2 = distCoeffs2.rows == 1 ? distCoeffs2.colRange(0, 5) : distCoeffs2.rowRange(0, 5);
  //    }
  //
  //    _Rmat.create(3, 3, rtype);
  //    _Tmat.create(3, 1, rtype);
  //
  //    Mat objPt, imgPt, imgPt2, npoints;
  //
  //    collectCalibrationData( _objectPoints, _imagePoints1, _imagePoints2,
  //                            objPt, imgPt, &imgPt2, npoints );
  //    CvMat c_objPt = objPt, c_imgPt = imgPt, c_imgPt2 = imgPt2, c_npoints = npoints;
  //    CvMat c_cameraMatrix1 = cameraMatrix1, c_distCoeffs1 = distCoeffs1;
  //    CvMat c_cameraMatrix2 = cameraMatrix2, c_distCoeffs2 = distCoeffs2;
  //    CvMat c_matR = _Rmat.getMat(), c_matT = _Tmat.getMat(), c_matE, c_matF, *p_matE = 0, *p_matF = 0;
  //
  //    if( _Emat.needed() )
  //    {
  //        _Emat.create(3, 3, rtype);
  //        p_matE = &(c_matE = _Emat.getMat());
  //    }
  //    if( _Fmat.needed() )
  //    {
  //        _Fmat.create(3, 3, rtype);
  //        p_matF = &(c_matF = _Fmat.getMat());
  //    }
  //
  //
  //double cvStereoCalibrate( const CvMat* _objectPoints, const CvMat* _imagePoints1,
  //                        const CvMat* _imagePoints2, const CvMat* _npoints,
  //                        CvMat* _cameraMatrix1, CvMat* _distCoeffs1,
  //                        CvMat* _cameraMatrix2, CvMat* _distCoeffs2,
  //                        CvSize imageSize, CvMat* matR, CvMat* matT,
  //                        CvMat* matE, CvMat* matF,
  //                        CvTermCriteria termCrit,
  //                        int flags )
  //{
  //    const int NINTRINSIC = 12;
  //    Ptr<CvMat> npoints, err, J_LR, Je, Ji, imagePoints[2], objectPoints, RT0;
  //    CvLevMarq solver;
  //    double reprojErr = 0;
  //
  //    double A[2][9], dk[2][8]={{0,0,0,0,0,0,0,0},{0,0,0,0,0,0,0,0}}, rlr[9];
  //    CvMat K[2], Dist[2], om_LR, T_LR;
  //    CvMat R_LR = cvMat(3, 3, CV_64F, rlr);
  //    int i, k, p, ni = 0, ofs, nimages, pointsTotal, maxPoints = 0;
  //    int nparams;
  //    bool recomputeIntrinsics = false;
  //    double aspectRatio[2] = {0,0};
  //
  //    CV_Assert( CV_IS_MAT(_imagePoints1) && CV_IS_MAT(_imagePoints2) &&
  //               CV_IS_MAT(_objectPoints) && CV_IS_MAT(_npoints) &&
  //               CV_IS_MAT(matR) && CV_IS_MAT(matT) );
  //
  //    CV_Assert( CV_ARE_TYPES_EQ(_imagePoints1, _imagePoints2) &&
  //               CV_ARE_DEPTHS_EQ(_imagePoints1, _objectPoints) );
  //
  //    CV_Assert( (_npoints->cols == 1 || _npoints->rows == 1) &&
  //               CV_MAT_TYPE(_npoints->type) == CV_32SC1 );
  //
  //    nimages = _npoints->cols + _npoints->rows - 1;
  //    npoints = cvCreateMat( _npoints->rows, _npoints->cols, _npoints->type );
  //    cvCopy( _npoints, npoints );
  //
  //    for( i = 0, pointsTotal = 0; i < nimages; i++ )
  //    {
  //        maxPoints = MAX(maxPoints, npoints->data.i[i]);
  //        pointsTotal += npoints->data.i[i];
  //    }
  //
  //    objectPoints = cvCreateMat( _objectPoints->rows, _objectPoints->cols,
  //                                CV_64FC(CV_MAT_CN(_objectPoints->type)));
  //    cvConvert( _objectPoints, objectPoints );
  //    cvReshape( objectPoints, objectPoints, 3, 1 );
  //
  //    for( k = 0; k < 2; k++ )
  //    {
  //        const CvMat* points = k == 0 ? _imagePoints1 : _imagePoints2;
  //        const CvMat* cameraMatrix = k == 0 ? _cameraMatrix1 : _cameraMatrix2;
  //        const CvMat* distCoeffs = k == 0 ? _distCoeffs1 : _distCoeffs2;
  //
  //        int cn = CV_MAT_CN(_imagePoints1->type);
  //        CV_Assert( (CV_MAT_DEPTH(_imagePoints1->type) == CV_32F ||
  //                CV_MAT_DEPTH(_imagePoints1->type) == CV_64F) &&
  //               ((_imagePoints1->rows == pointsTotal && _imagePoints1->cols*cn == 2) ||
  //                (_imagePoints1->rows == 1 && _imagePoints1->cols == pointsTotal && cn == 2)) );
  //
  //        K[k] = cvMat(3,3,CV_64F,A[k]);
  //        Dist[k] = cvMat(1,8,CV_64F,dk[k]);
  //
  //        imagePoints[k] = cvCreateMat( points->rows, points->cols, CV_64FC(CV_MAT_CN(points->type)));
  //        cvConvert( points, imagePoints[k] );
  //        cvReshape( imagePoints[k], imagePoints[k], 2, 1 );
  //
  //        if( flags & (CV_CALIB_FIX_INTRINSIC|CV_CALIB_USE_INTRINSIC_GUESS|
  //            CV_CALIB_FIX_ASPECT_RATIO|CV_CALIB_FIX_FOCAL_LENGTH) )
  //            cvConvert( cameraMatrix, &K[k] );
  //
  //        if( flags & (CV_CALIB_FIX_INTRINSIC|CV_CALIB_USE_INTRINSIC_GUESS|
  //            CV_CALIB_FIX_K1|CV_CALIB_FIX_K2|CV_CALIB_FIX_K3|CV_CALIB_FIX_K4|CV_CALIB_FIX_K5|CV_CALIB_FIX_K6) )
  //        {
  //            CvMat tdist = cvMat( distCoeffs->rows, distCoeffs->cols,
  //                CV_MAKETYPE(CV_64F,CV_MAT_CN(distCoeffs->type)), Dist[k].data.db );
  //            cvConvert( distCoeffs, &tdist );
  //        }
  //
  //        if( !(flags & (CV_CALIB_FIX_INTRINSIC|CV_CALIB_USE_INTRINSIC_GUESS)))
  //        {
  //            cvCalibrateCamera2( objectPoints, imagePoints[k],
  //                npoints, imageSize, &K[k], &Dist[k], 0, 0, flags );
  //        }
  //    }
  //
  //    if( flags & CV_CALIB_SAME_FOCAL_LENGTH )
  //    {
  //        static const int avg_idx[] = { 0, 4, 2, 5, -1 };
  //        for( k = 0; avg_idx[k] >= 0; k++ )
  //            A[0][avg_idx[k]] = A[1][avg_idx[k]] = (A[0][avg_idx[k]] + A[1][avg_idx[k]])*0.5;
  //    }
  //
  //    if( flags & CV_CALIB_FIX_ASPECT_RATIO )
  //    {
  //        for( k = 0; k < 2; k++ )
  //            aspectRatio[k] = A[k][0]/A[k][4];
  //    }
  //
  //    recomputeIntrinsics = (flags & CV_CALIB_FIX_INTRINSIC) == 0;
  //
  //    err = cvCreateMat( maxPoints*2, 1, CV_64F );
  //    Je = cvCreateMat( maxPoints*2, 6, CV_64F );
  //    J_LR = cvCreateMat( maxPoints*2, 6, CV_64F );
  //    Ji = cvCreateMat( maxPoints*2, NINTRINSIC, CV_64F );
  //    cvZero( Ji );
  //
  //    // we optimize for the inter-camera R(3),t(3), then, optionally,
  //    // for intrinisic parameters of each camera ((fx,fy,cx,cy,k1,k2,p1,p2) ~ 8 parameters).
  //    nparams = 6*(nimages+1) + (recomputeIntrinsics ? NINTRINSIC*2 : 0);
  //
  //    // storage for initial [om(R){i}|t{i}] (in order to compute the median for each component)
  //    RT0 = cvCreateMat( 6, nimages, CV_64F );
  //
  //    solver.init( nparams, 0, termCrit );
  //    if( recomputeIntrinsics )
  //    {
  //        uchar* imask = solver.mask->data.ptr + nparams - NINTRINSIC*2;
  //        if( !(flags & CV_CALIB_RATIONAL_MODEL) )
  //            flags |= CV_CALIB_FIX_K4 | CV_CALIB_FIX_K5 | CV_CALIB_FIX_K6;
  //        if( flags & CV_CALIB_FIX_ASPECT_RATIO )
  //            imask[0] = imask[NINTRINSIC] = 0;
  //        if( flags & CV_CALIB_FIX_FOCAL_LENGTH )
  //            imask[0] = imask[1] = imask[NINTRINSIC] = imask[NINTRINSIC+1] = 0;
  //        if( flags & CV_CALIB_FIX_PRINCIPAL_POINT )
  //            imask[2] = imask[3] = imask[NINTRINSIC+2] = imask[NINTRINSIC+3] = 0;
  //        if( flags & CV_CALIB_ZERO_TANGENT_DIST )
  //            imask[6] = imask[7] = imask[NINTRINSIC+6] = imask[NINTRINSIC+7] = 0;
  //        if( flags & CV_CALIB_FIX_K1 )
  //            imask[4] = imask[NINTRINSIC+4] = 0;
  //        if( flags & CV_CALIB_FIX_K2 )
  //            imask[5] = imask[NINTRINSIC+5] = 0;
  //        if( flags & CV_CALIB_FIX_K3 )
  //            imask[8] = imask[NINTRINSIC+8] = 0;
  //        if( flags & CV_CALIB_FIX_K4 )
  //            imask[9] = imask[NINTRINSIC+9] = 0;
  //        if( flags & CV_CALIB_FIX_K5 )
  //            imask[10] = imask[NINTRINSIC+10] = 0;
  //        if( flags & CV_CALIB_FIX_K6 )
  //            imask[11] = imask[NINTRINSIC+11] = 0;
  //    }
  //
  //    /*
  //       Compute initial estimate of pose
  //
  //       For each image, compute:
  //          R(om) is the rotation matrix of om
  //          om(R) is the rotation vector of R
  //          R_ref = R(om_right) * R(om_left)'
  //          T_ref_list = [T_ref_list; T_right - R_ref * T_left]
  //          om_ref_list = {om_ref_list; om(R_ref)]
  //
  //       om = median(om_ref_list)
  //       T = median(T_ref_list)
  //    */
  //    for( i = ofs = 0; i < nimages; ofs += ni, i++ )
  //    {
  //        ni = npoints->data.i[i];
  //        CvMat objpt_i;
  //        double _om[2][3], r[2][9], t[2][3];
  //        CvMat om[2], R[2], T[2], imgpt_i[2];
  //
  //        objpt_i = cvMat(1, ni, CV_64FC3, objectPoints->data.db + ofs*3);
  //        for( k = 0; k < 2; k++ )
  //        {
  //            imgpt_i[k] = cvMat(1, ni, CV_64FC2, imagePoints[k]->data.db + ofs*2);
  //            om[k] = cvMat(3, 1, CV_64F, _om[k]);
  //            R[k] = cvMat(3, 3, CV_64F, r[k]);
  //            T[k] = cvMat(3, 1, CV_64F, t[k]);
  //
  //            // FIXME: here we ignore activePoints[k] because of
  //            // the limited API of cvFindExtrnisicCameraParams2
  //            cvFindExtrinsicCameraParams2( &objpt_i, &imgpt_i[k], &K[k], &Dist[k], &om[k], &T[k] );
  //            cvRodrigues2( &om[k], &R[k] );
  //            if( k == 0 )
  //            {
  //                // save initial om_left and T_left
  //                solver.param->data.db[(i+1)*6] = _om[0][0];
  //                solver.param->data.db[(i+1)*6 + 1] = _om[0][1];
  //                solver.param->data.db[(i+1)*6 + 2] = _om[0][2];
  //                solver.param->data.db[(i+1)*6 + 3] = t[0][0];
  //                solver.param->data.db[(i+1)*6 + 4] = t[0][1];
  //                solver.param->data.db[(i+1)*6 + 5] = t[0][2];
  //            }
  //        }
  //        cvGEMM( &R[1], &R[0], 1, 0, 0, &R[0], CV_GEMM_B_T );
  //        cvGEMM( &R[0], &T[0], -1, &T[1], 1, &T[1] );
  //        cvRodrigues2( &R[0], &T[0] );
  //        RT0->data.db[i] = t[0][0];
  //        RT0->data.db[i + nimages] = t[0][1];
  //        RT0->data.db[i + nimages*2] = t[0][2];
  //        RT0->data.db[i + nimages*3] = t[1][0];
  //        RT0->data.db[i + nimages*4] = t[1][1];
  //        RT0->data.db[i + nimages*5] = t[1][2];
  //    }
  //
  //    // find the medians and save the first 6 parameters
  //    for( i = 0; i < 6; i++ )
  //    {
  //        qsort( RT0->data.db + i*nimages, nimages, CV_ELEM_SIZE(RT0->type), dbCmp );
  //        solver.param->data.db[i] = nimages % 2 != 0 ? RT0->data.db[i*nimages + nimages/2] :
  //            (RT0->data.db[i*nimages + nimages/2 - 1] + RT0->data.db[i*nimages + nimages/2])*0.5;
  //    }
  //
  //    if( recomputeIntrinsics )
  //        for( k = 0; k < 2; k++ )
  //        {
  //            double* iparam = solver.param->data.db + (nimages+1)*6 + k*NINTRINSIC;
  //            if( flags & CV_CALIB_ZERO_TANGENT_DIST )
  //                dk[k][2] = dk[k][3] = 0;
  //            iparam[0] = A[k][0]; iparam[1] = A[k][4]; iparam[2] = A[k][2]; iparam[3] = A[k][5];
  //            iparam[4] = dk[k][0]; iparam[5] = dk[k][1]; iparam[6] = dk[k][2];
  //            iparam[7] = dk[k][3]; iparam[8] = dk[k][4]; iparam[9] = dk[k][5];
  //            iparam[10] = dk[k][6]; iparam[11] = dk[k][7];
  //        }
  //
  //    om_LR = cvMat(3, 1, CV_64F, solver.param->data.db);
  //    T_LR = cvMat(3, 1, CV_64F, solver.param->data.db + 3);
  //
  //    for(;;)
  //    {
  //        const CvMat* param = 0;
  //        CvMat tmpimagePoints;
  //        CvMat *JtJ = 0, *JtErr = 0;
  //        double *_errNorm = 0;
  //        double _omR[3], _tR[3];
  //        double _dr3dr1[9], _dr3dr2[9], /*_dt3dr1[9],*/ _dt3dr2[9], _dt3dt1[9], _dt3dt2[9];
  //        CvMat dr3dr1 = cvMat(3, 3, CV_64F, _dr3dr1);
  //        CvMat dr3dr2 = cvMat(3, 3, CV_64F, _dr3dr2);
  //        //CvMat dt3dr1 = cvMat(3, 3, CV_64F, _dt3dr1);
  //        CvMat dt3dr2 = cvMat(3, 3, CV_64F, _dt3dr2);
  //        CvMat dt3dt1 = cvMat(3, 3, CV_64F, _dt3dt1);
  //        CvMat dt3dt2 = cvMat(3, 3, CV_64F, _dt3dt2);
  //        CvMat om[2], T[2], imgpt_i[2];
  //        CvMat dpdrot_hdr, dpdt_hdr, dpdf_hdr, dpdc_hdr, dpdk_hdr;
  //        CvMat *dpdrot = &dpdrot_hdr, *dpdt = &dpdt_hdr, *dpdf = 0, *dpdc = 0, *dpdk = 0;
  //
  //        if( !solver.updateAlt( param, JtJ, JtErr, _errNorm ))
  //            break;
  //        reprojErr = 0;
  //
  //        cvRodrigues2( &om_LR, &R_LR );
  //        om[1] = cvMat(3,1,CV_64F,_omR);
  //        T[1] = cvMat(3,1,CV_64F,_tR);
  //
  //        if( recomputeIntrinsics )
  //        {
  //            double* iparam = solver.param->data.db + (nimages+1)*6;
  //            double* ipparam = solver.prevParam->data.db + (nimages+1)*6;
  //            dpdf = &dpdf_hdr;
  //            dpdc = &dpdc_hdr;
  //            dpdk = &dpdk_hdr;
  //            if( flags & CV_CALIB_SAME_FOCAL_LENGTH )
  //            {
  //                iparam[NINTRINSIC] = iparam[0];
  //                iparam[NINTRINSIC+1] = iparam[1];
  //                ipparam[NINTRINSIC] = ipparam[0];
  //                ipparam[NINTRINSIC+1] = ipparam[1];
  //            }
  //            if( flags & CV_CALIB_FIX_ASPECT_RATIO )
  //            {
  //                iparam[0] = iparam[1]*aspectRatio[0];
  //                iparam[NINTRINSIC] = iparam[NINTRINSIC+1]*aspectRatio[1];
  //                ipparam[0] = ipparam[1]*aspectRatio[0];
  //                ipparam[NINTRINSIC] = ipparam[NINTRINSIC+1]*aspectRatio[1];
  //            }
  //            for( k = 0; k < 2; k++ )
  //            {
  //                A[k][0] = iparam[k*NINTRINSIC+0];
  //                A[k][4] = iparam[k*NINTRINSIC+1];
  //                A[k][2] = iparam[k*NINTRINSIC+2];
  //                A[k][5] = iparam[k*NINTRINSIC+3];
  //                dk[k][0] = iparam[k*NINTRINSIC+4];
  //                dk[k][1] = iparam[k*NINTRINSIC+5];
  //                dk[k][2] = iparam[k*NINTRINSIC+6];
  //                dk[k][3] = iparam[k*NINTRINSIC+7];
  //                dk[k][4] = iparam[k*NINTRINSIC+8];
  //                dk[k][5] = iparam[k*NINTRINSIC+9];
  //                dk[k][6] = iparam[k*NINTRINSIC+10];
  //                dk[k][7] = iparam[k*NINTRINSIC+11];
  //            }
  //        }
  //
  //        for( i = ofs = 0; i < nimages; ofs += ni, i++ )
  //        {
  //            ni = npoints->data.i[i];
  //            CvMat objpt_i, _part;
  //
  //            om[0] = cvMat(3,1,CV_64F,solver.param->data.db+(i+1)*6);
  //            T[0] = cvMat(3,1,CV_64F,solver.param->data.db+(i+1)*6+3);
  //
  //            if( JtJ || JtErr )
  //                cvComposeRT( &om[0], &T[0], &om_LR, &T_LR, &om[1], &T[1], &dr3dr1, 0,
  //                             &dr3dr2, 0, 0, &dt3dt1, &dt3dr2, &dt3dt2 );
  //            else
  //                cvComposeRT( &om[0], &T[0], &om_LR, &T_LR, &om[1], &T[1] );
  //
  //            objpt_i = cvMat(1, ni, CV_64FC3, objectPoints->data.db + ofs*3);
  //            err->rows = Je->rows = J_LR->rows = Ji->rows = ni*2;
  //            cvReshape( err, &tmpimagePoints, 2, 1 );
  //
  //            cvGetCols( Ji, &dpdf_hdr, 0, 2 );
  //            cvGetCols( Ji, &dpdc_hdr, 2, 4 );
  //            cvGetCols( Ji, &dpdk_hdr, 4, NINTRINSIC );
  //            cvGetCols( Je, &dpdrot_hdr, 0, 3 );
  //            cvGetCols( Je, &dpdt_hdr, 3, 6 );
  //
  //            for( k = 0; k < 2; k++ )
  //            {
  //                double l2err;
  //                imgpt_i[k] = cvMat(1, ni, CV_64FC2, imagePoints[k]->data.db + ofs*2);
  //
  //                if( JtJ || JtErr )
  //                    cvProjectPoints2( &objpt_i, &om[k], &T[k], &K[k], &Dist[k],
  //                            &tmpimagePoints, dpdrot, dpdt, dpdf, dpdc, dpdk,
  //                            (flags & CV_CALIB_FIX_ASPECT_RATIO) ? aspectRatio[k] : 0);
  //                else
  //                    cvProjectPoints2( &objpt_i, &om[k], &T[k], &K[k], &Dist[k], &tmpimagePoints );
  //                cvSub( &tmpimagePoints, &imgpt_i[k], &tmpimagePoints );
  //
  //                l2err = cvNorm( &tmpimagePoints, 0, CV_L2 );
  //
  //                if( JtJ || JtErr )
  //                {
  //                    int iofs = (nimages+1)*6 + k*NINTRINSIC, eofs = (i+1)*6;
  //                    assert( JtJ && JtErr );
  //
  //                    if( k == 1 )
  //                    {
  //                        // d(err_{x|y}R) ~ de3
  //                        // convert de3/{dr3,dt3} => de3{dr1,dt1} & de3{dr2,dt2}
  //                        for( p = 0; p < ni*2; p++ )
  //                        {
  //                            CvMat de3dr3 = cvMat( 1, 3, CV_64F, Je->data.ptr + Je->step*p );
  //                            CvMat de3dt3 = cvMat( 1, 3, CV_64F, de3dr3.data.db + 3 );
  //                            CvMat de3dr2 = cvMat( 1, 3, CV_64F, J_LR->data.ptr + J_LR->step*p );
  //                            CvMat de3dt2 = cvMat( 1, 3, CV_64F, de3dr2.data.db + 3 );
  //                            double _de3dr1[3], _de3dt1[3];
  //                            CvMat de3dr1 = cvMat( 1, 3, CV_64F, _de3dr1 );
  //                            CvMat de3dt1 = cvMat( 1, 3, CV_64F, _de3dt1 );
  //
  //                            cvMatMul( &de3dr3, &dr3dr1, &de3dr1 );
  //                            cvMatMul( &de3dt3, &dt3dt1, &de3dt1 );
  //
  //                            cvMatMul( &de3dr3, &dr3dr2, &de3dr2 );
  //                            cvMatMulAdd( &de3dt3, &dt3dr2, &de3dr2, &de3dr2 );
  //
  //                            cvMatMul( &de3dt3, &dt3dt2, &de3dt2 );
  //
  //                            cvCopy( &de3dr1, &de3dr3 );
  //                            cvCopy( &de3dt1, &de3dt3 );
  //                        }
  //
  //                        cvGetSubRect( JtJ, &_part, cvRect(0, 0, 6, 6) );
  //                        cvGEMM( J_LR, J_LR, 1, &_part, 1, &_part, CV_GEMM_A_T );
  //
  //                        cvGetSubRect( JtJ, &_part, cvRect(eofs, 0, 6, 6) );
  //                        cvGEMM( J_LR, Je, 1, 0, 0, &_part, CV_GEMM_A_T );
  //
  //                        cvGetRows( JtErr, &_part, 0, 6 );
  //                        cvGEMM( J_LR, err, 1, &_part, 1, &_part, CV_GEMM_A_T );
  //                    }
  //
  //                    cvGetSubRect( JtJ, &_part, cvRect(eofs, eofs, 6, 6) );
  //                    cvGEMM( Je, Je, 1, &_part, 1, &_part, CV_GEMM_A_T );
  //
  //                    cvGetRows( JtErr, &_part, eofs, eofs + 6 );
  //                    cvGEMM( Je, err, 1, &_part, 1, &_part, CV_GEMM_A_T );
  //
  //                    if( recomputeIntrinsics )
  //                    {
  //                        cvGetSubRect( JtJ, &_part, cvRect(iofs, iofs, NINTRINSIC, NINTRINSIC) );
  //                        cvGEMM( Ji, Ji, 1, &_part, 1, &_part, CV_GEMM_A_T );
  //                        cvGetSubRect( JtJ, &_part, cvRect(iofs, eofs, NINTRINSIC, 6) );
  //                        cvGEMM( Je, Ji, 1, &_part, 1, &_part, CV_GEMM_A_T );
  //                        if( k == 1 )
  //                        {
  //                            cvGetSubRect( JtJ, &_part, cvRect(iofs, 0, NINTRINSIC, 6) );
  //                            cvGEMM( J_LR, Ji, 1, &_part, 1, &_part, CV_GEMM_A_T );
  //                        }
  //                        cvGetRows( JtErr, &_part, iofs, iofs + NINTRINSIC );
  //                        cvGEMM( Ji, err, 1, &_part, 1, &_part, CV_GEMM_A_T );
  //                    }
  //                }
  //
  //                reprojErr += l2err*l2err;
  //            }
  //        }
  //        if(_errNorm)
  //            *_errNorm = reprojErr;
  //    }
  //
  //    cvRodrigues2( &om_LR, &R_LR );
  //    if( matR->rows == 1 || matR->cols == 1 )
  //        cvConvert( &om_LR, matR );
  //    else
  //        cvConvert( &R_LR, matR );
  //    cvConvert( &T_LR, matT );
  //
  //    if( recomputeIntrinsics )
  //    {
  //        cvConvert( &K[0], _cameraMatrix1 );
  //        cvConvert( &K[1], _cameraMatrix2 );
  //
  //        for( k = 0; k < 2; k++ )
  //        {
  //            CvMat* distCoeffs = k == 0 ? _distCoeffs1 : _distCoeffs2;
  //            CvMat tdist = cvMat( distCoeffs->rows, distCoeffs->cols,
  //                CV_MAKETYPE(CV_64F,CV_MAT_CN(distCoeffs->type)), Dist[k].data.db );
  //            cvConvert( &tdist, distCoeffs );
  //        }
  //    }
  //
  //    if( matE || matF )
  //    {
  //        double* t = T_LR.data.db;
  //        double tx[] =
  //        {
  //            0, -t[2], t[1],
  //            t[2], 0, -t[0],
  //            -t[1], t[0], 0
  //        };
  //        CvMat Tx = cvMat(3, 3, CV_64F, tx);
  //        double e[9], f[9];
  //        CvMat E = cvMat(3, 3, CV_64F, e);
  //        CvMat F = cvMat(3, 3, CV_64F, f);
  //        cvMatMul( &Tx, &R_LR, &E );
  //        if( matE )
  //            cvConvert( &E, matE );
  //        if( matF )
  //        {
  //            double ik[9];
  //            CvMat iK = cvMat(3, 3, CV_64F, ik);
  //            cvInvert(&K[1], &iK);
  //            cvGEMM( &iK, &E, 1, 0, 0, &E, CV_GEMM_A_T );
  //            cvInvert(&K[0], &iK);
  //            cvMatMul(&E, &iK, &F);
  //            cvConvertScale( &F, matF, fabs(f[8]) > 0 ? 1./f[8] : 1 );
  //        }
  //    }
  //
  //    return std::sqrt(reprojErr/(pointsTotal*2));
  //}
  //
  ////    double err = cvStereoCalibrate(&c_objPt, &c_imgPt, &c_imgPt2, &c_npoints, &c_cameraMatrix1,
  ////        &c_distCoeffs1, &c_cameraMatrix2, &c_distCoeffs2, imageSize,
  ////        &c_matR, &c_matT, p_matE, p_matF, criteria, flags );
  ////
  //    cameraMatrix1.copyTo(_cameraMatrix1);
  //    cameraMatrix2.copyTo(_cameraMatrix2);
  //    distCoeffs1.copyTo(_distCoeffs1);
  //    distCoeffs2.copyTo(_distCoeffs2);
  //
  //    return err;
  //}
  //
  //
  //void cv::stereoRectify( InputArray _cameraMatrix1, InputArray _distCoeffs1,
  //                        InputArray _cameraMatrix2, InputArray _distCoeffs2,
  //                        Size imageSize, InputArray _Rmat, InputArray _Tmat,
  //                        OutputArray _Rmat1, OutputArray _Rmat2,
  //                        OutputArray _Pmat1, OutputArray _Pmat2,
  //                        OutputArray _Qmat, int flags,
  //                        double alpha, Size newImageSize,
  //                        Rect* validPixROI1, Rect* validPixROI2 )
  //{
  //    Mat cameraMatrix1 = _cameraMatrix1.getMat(), cameraMatrix2 = _cameraMatrix2.getMat();
  //    Mat distCoeffs1 = _distCoeffs1.getMat(), distCoeffs2 = _distCoeffs2.getMat();
  //    Mat Rmat = _Rmat.getMat(), Tmat = _Tmat.getMat();
  //    CvMat c_cameraMatrix1 = cameraMatrix1;
  //    CvMat c_cameraMatrix2 = cameraMatrix2;
  //    CvMat c_distCoeffs1 = distCoeffs1;
  //    CvMat c_distCoeffs2 = distCoeffs2;
  //    CvMat c_R = Rmat, c_T = Tmat;
  //
  //    int rtype = CV_64F;
  //    _Rmat1.create(3, 3, rtype);
  //    _Rmat2.create(3, 3, rtype);
  //    _Pmat1.create(3, 4, rtype);
  //    _Pmat2.create(3, 4, rtype);
  //    CvMat c_R1 = _Rmat1.getMat(), c_R2 = _Rmat2.getMat(), c_P1 = _Pmat1.getMat(), c_P2 = _Pmat2.getMat();
  //    CvMat c_Q, *p_Q = 0;
  //
  //    if( _Qmat.needed() )
  //    {
  //        _Qmat.create(4, 4, rtype);
  //        p_Q = &(c_Q = _Qmat.getMat());
  //    }
  //
  //    cvStereoRectify( &c_cameraMatrix1, &c_cameraMatrix2, &c_distCoeffs1, &c_distCoeffs2,
  //        imageSize, &c_R, &c_T, &c_R1, &c_R2, &c_P1, &c_P2, p_Q, flags, alpha,
  //        newImageSize, (CvRect*)validPixROI1, (CvRect*)validPixROI2);
  //}
  //
  //bool cv::stereoRectifyUncalibrated( InputArray _points1, InputArray _points2,
  //                                    InputArray _Fmat, Size imgSize,
  //                                    OutputArray _Hmat1, OutputArray _Hmat2, double threshold )
  //{
  //    int rtype = CV_64F;
  //    _Hmat1.create(3, 3, rtype);
  //    _Hmat2.create(3, 3, rtype);
  //    Mat F = _Fmat.getMat();
  //    Mat points1 = _points1.getMat(), points2 = _points2.getMat();
  //    CvMat c_pt1 = points1, c_pt2 = points2;
  //    CvMat c_F, *p_F=0, c_H1 = _Hmat1.getMat(), c_H2 = _Hmat2.getMat();
  //    if( F.size() == Size(3, 3) )
  //        p_F = &(c_F = F);
  //    return cvStereoRectifyUncalibrated(&c_pt1, &c_pt2, p_F, imgSize, &c_H1, &c_H2, threshold) > 0;

