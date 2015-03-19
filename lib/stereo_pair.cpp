
namespace Distortion {
double cv::stereoCalibrate( InputArrayOfArrays _objectPoints,
                          InputArrayOfArrays _imagePoints1,
                          InputArrayOfArrays _imagePoints2,
                          InputOutputArray _cameraMatrix1, InputOutputArray _distCoeffs1,
                          InputOutputArray _cameraMatrix2, InputOutputArray _distCoeffs2,
                          Size imageSize, OutputArray _Rmat, OutputArray _Tmat,
                          OutputArray _Emat, OutputArray _Fmat, TermCriteria criteria,
                          int flags )
{
    int rtype = CV_64F;
    Mat cameraMatrix1 = _cameraMatrix1.getMat();
    Mat cameraMatrix2 = _cameraMatrix2.getMat();
    Mat distCoeffs1 = _distCoeffs1.getMat();
    Mat distCoeffs2 = _distCoeffs2.getMat();
    cameraMatrix1 = prepareCameraMatrix(cameraMatrix1, rtype);
    cameraMatrix2 = prepareCameraMatrix(cameraMatrix2, rtype);
    distCoeffs1 = prepareDistCoeffs(distCoeffs1, rtype);
    distCoeffs2 = prepareDistCoeffs(distCoeffs2, rtype);

    if( !(flags & CALIB_RATIONAL_MODEL) )
    {
        distCoeffs1 = distCoeffs1.rows == 1 ? distCoeffs1.colRange(0, 5) : distCoeffs1.rowRange(0, 5);
        distCoeffs2 = distCoeffs2.rows == 1 ? distCoeffs2.colRange(0, 5) : distCoeffs2.rowRange(0, 5);
    }

    _Rmat.create(3, 3, rtype);
    _Tmat.create(3, 1, rtype);

    Mat objPt, imgPt, imgPt2, npoints;

    collectCalibrationData( _objectPoints, _imagePoints1, _imagePoints2,
                            objPt, imgPt, &imgPt2, npoints );
    CvMat c_objPt = objPt, c_imgPt = imgPt, c_imgPt2 = imgPt2, c_npoints = npoints;
    CvMat c_cameraMatrix1 = cameraMatrix1, c_distCoeffs1 = distCoeffs1;
    CvMat c_cameraMatrix2 = cameraMatrix2, c_distCoeffs2 = distCoeffs2;
    CvMat c_matR = _Rmat.getMat(), c_matT = _Tmat.getMat(), c_matE, c_matF, *p_matE = 0, *p_matF = 0;

    if( _Emat.needed() )
    {
        _Emat.create(3, 3, rtype);
        p_matE = &(c_matE = _Emat.getMat());
    }
    if( _Fmat.needed() )
    {
        _Fmat.create(3, 3, rtype);
        p_matF = &(c_matF = _Fmat.getMat());
    }

    double err = cvStereoCalibrate(&c_objPt, &c_imgPt, &c_imgPt2, &c_npoints, &c_cameraMatrix1,
        &c_distCoeffs1, &c_cameraMatrix2, &c_distCoeffs2, imageSize,
        &c_matR, &c_matT, p_matE, p_matF, criteria, flags );

    cameraMatrix1.copyTo(_cameraMatrix1);
    cameraMatrix2.copyTo(_cameraMatrix2);
    distCoeffs1.copyTo(_distCoeffs1);
    distCoeffs2.copyTo(_distCoeffs2);

    return err;
}
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


}
