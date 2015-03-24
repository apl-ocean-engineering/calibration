
#include "distortion_model.h"

namespace Distortion {

  using namespace std;
  using namespace cv;


  // Maintain the notation that the entry at (0,1) is alpha * fx
  PinholeCamera::PinholeCamera( void )
    : _fx(1.0), _fy(1.0), _alpha(0.0), _cx(0.0), _cy(0.0)
  {;}

  PinholeCamera::PinholeCamera( const Matx33d &k )
    : _fx( k(0,0) ), _fy( k(1,1) ), _alpha( k(0,1)/_fx ), 
    _cx( k(0,2) ), _cy( k(1,2) )
  {;}

  PinholeCamera::PinholeCamera( const Mat &k )
  { Matx33d kMatx;
    k.convertTo( kMatx, CV_64F );
    setCamera( kMatx ); 
  }

  void PinholeCamera::setCamera( const Matx33d &k )
  {
    setCamera( k(0,0), k(1,1), k(0,2), k(1,2), k(0,1)/k(0,0) );
  }

  void PinholeCamera::setCamera( double fx, double fy, double cx, double cy, double alpha )
  {
    _fx = fx;
    _fy = fy;
    _cx = cx;
    _cy = cy;
    _alpha = alpha;
  }

  Matx33d PinholeCamera::matx( void ) const 
  {
    return Matx33d( _fx, _alpha*_fx, _cx, 0., _fy, _cy, 0., 0., 1. );
  }

  Mat PinholeCamera::mat( void ) const
  { return Mat(matx()); }

  ImagePoint PinholeCamera::image( const ImagePoint &xd ) const
  {
    return  ImagePoint( _fx * ( xd[0] + _alpha*xd[1] ) + _cx,
        _fy *   xd[1]                    + _cy );
  }

  ImagePoint PinholeCamera::normalize( const ImagePoint &pt ) const
  {
    return ImagePoint( 1.0 / _fx * (pt[0] - _cx ),
        1.0 / _fy * (pt[1] - _cy ) );
  }

  FileStorage &PinholeCamera::write( FileStorage &out ) const
  {
    out << "camera_model" << name();
    out << "camera_matrix" << mat();
    return out;
  }

  Mat PinholeCamera::getOptimalNewCameraMatrix( const Size &imgSize, 
      double alpha, const Size &newImgSize,
      Rect &validPixROI, bool centerPrincipalPoint )
  {

    Rect_<float> inner, outer;
    Size theImgSize = (newImgSize.width*newImgSize.height != 0 ) ? newImgSize : imgSize;

    double M[3][3];
    Mat matM(3, 3, CV_64F, M);
    mat().copyTo( matM );

    if( centerPrincipalPoint )
    {
      double cx0 = M[0][2];
      double cy0 = M[1][2];
      double cx = (theImgSize.width-1)*0.5;
      double cy = (theImgSize.height-1)*0.5;

      getRectangles( Mat(), mat(), imgSize, inner, outer );
      double s0 = std::max(std::max(std::max((double)cx/(cx0 - inner.x), (double)cy/(cy0 - inner.y)),
            (double)cx/(inner.x + inner.width - cx0)),
          (double)cy/(inner.y + inner.height - cy0));
      double s1 = std::min(std::min(std::min((double)cx/(cx0 - outer.x), (double)cy/(cy0 - outer.y)),
            (double)cx/(outer.x + outer.width - cx0)),
          (double)cy/(outer.y + outer.height - cy0));
      double s = s0*(1 - alpha) + s1*alpha;

      M[0][0] *= s;
      M[1][1] *= s;
      M[0][2] = cx;
      M[1][2] = cy;

      inner = Rect_<float>((float)((inner.x - cx0)*s + cx),
          (float)((inner.y - cy0)*s + cy),
          (float)(inner.width*s),
          (float)(inner.height*s));

      Rect r( cvCeil(inner.x), cvCeil(inner.y), cvFloor(inner.width), cvFloor(inner.height));
      r &= Rect(0, 0, theImgSize.width, theImgSize.height);

      validPixROI = r;
    }
    else
    {
      // Get inscribed and circumscribed rectangles in normalized
      // (independent of camera matrix) coordinates
      getRectangles( Mat(), Mat(), imgSize, inner, outer );

      // Projection mapping inner rectangle to viewport
      double fx0 = (theImgSize.width  - 1) / inner.width;
      double fy0 = (theImgSize.height - 1) / inner.height;
      double cx0 = -fx0 * inner.x;
      double cy0 = -fy0 * inner.y;

      // Projection mapping outer rectangle to viewport
      double fx1 = (theImgSize.width  - 1) / outer.width;
      double fy1 = (theImgSize.height - 1) / outer.height;
      double cx1 = -fx1 * outer.x;
      double cy1 = -fy1 * outer.y;

      // Interpolate between the two optimal projections
      M[0][0] = fx0*(1 - alpha) + fx1*alpha;
      M[1][1] = fy0*(1 - alpha) + fy1*alpha;
      M[0][2] = cx0*(1 - alpha) + cx1*alpha;
      M[1][2] = cy0*(1 - alpha) + cy1*alpha;

      getRectangles(  Mat(), matM, imgSize, inner, outer );
      Rect r = inner;
      r &= cv::Rect(0, 0, theImgSize.width, theImgSize.height);
      validPixROI = r;
    }

    // Because the underlying data is actually M[][], it
    // goes undefined when the function exits
    Mat output;
    matM.copyTo( output );
    return output;
  }


  void PinholeCamera::getRectangles(
      const Mat &R, const Mat &newCameraMatrix, const Size &imgSize,
      Rect_<float>& inner, Rect_<float>& outer ) const
  {
    const int N = 9;
    int x, y, k;
    ImagePointsVec pts;

    for( y = k = 0; y < N; y++ )
      for( x = 0; x < N; x++ )
        pts.push_back( ImagePoint((float)x*imgSize.width/(N-1),
              (float)y*imgSize.height/(N-1)) );

    undistortPoints(pts, pts, R, newCameraMatrix);

    float iX0=-FLT_MAX, iX1=FLT_MAX, iY0=-FLT_MAX, iY1=FLT_MAX;
    float oX0=FLT_MAX, oX1=-FLT_MAX, oY0=FLT_MAX, oY1=-FLT_MAX;
    // find the inscribed rectangle.
    // the code will likely not work with extreme rotation matrices (R) (>45%)
    for( y = k = 0; y < N; y++ )
      for( x = 0; x < N; x++ )
      {
        ImagePoint &p = pts[ y*N + x ];
        oX0 = MIN(oX0, p[0]);
        oX1 = MAX(oX1, p[0]);
        oY0 = MIN(oY0, p[1]);
        oY1 = MAX(oY1, p[1]);

        if( x == 0 )
          iX0 = MAX(iX0, p[0]);
        if( x == N-1 )
          iX1 = MIN(iX1, p[0]);
        if( y == 0 )
          iY0 = MAX(iY0, p[1]);
        if( y == N-1 )
          iY1 = MIN(iY1, p[1]);
      }
    inner = Rect_<float>(iX0, iY0, iX1-iX0, iY1-iY0);
    outer = Rect_<float>(oX0, oY0, oX1-oX0, oY1-oY0);
  }

  struct TxReprojector {
    TxReprojector( const Matx33d &mat ) : _mat( mat ) {;}
    Matx33d _mat;

    ImagePoint operator()( const ImagePoint &pt )
    {
      Vec3d pth( pt[0], pt[1], 1.0 );
      Vec3d out = _mat * pth;
      return ImagePoint( out[0]/out[2], out[1]/out[2] );
    }
  };



  void PinholeCamera::undistortPoints( const ImagePointsVec &distorted, 
      ImagePointsVec &undistorted, 
      const Mat &R, const Mat &P) const
  {
    // will support only 2-channel data now for points
    undistorted.resize(distorted.size());

    CV_Assert(P.empty() || P.size() == Size(3, 3) || P.size() == Size(4, 3));
    CV_Assert(R.empty() || R.size() == Size(3, 3) ); //|| R.total() * R.channels() == 3);

    // I believe the original supported axis-angle rotation vectors as well
    cv::Matx33d RR( Matx33d::eye() );
    if (!R.empty() && R.total() * R.channels() == 3)
    {
      cv::Vec3d rvec;
      R.convertTo(rvec, CV_64F);
      RR = cv::Affine3d(rvec).rotation();
    }
    else if (!R.empty() && R.size() == Size(3, 3))
      R.convertTo(RR, CV_64F);

    if(!P.empty())
    {
      cv::Matx33d PP;
      P.colRange(0, 3).convertTo(PP, CV_64F);
      RR = PP * RR;
    }

    undistorted = unwarp( normalize(distorted ) );

    std::transform( undistorted.begin(), undistorted.end(), undistorted.begin(),
        TxReprojector( RR ) );


    //    for(size_t i = 0; i < distorted.size(); i++ )
    //    {
    //      Vec2d pw( unimage( distorted[i] ) );
    //      Vec2d pu( undistort( pw ) );
    //
    //      // reproject
    //      Vec3d pr = RR * Vec3d(pu[0], pu[1], 1.0); // rotated point optionally multiplied by new camera matrix
    //      Vec2d fi(pr[0]/pr[2], pr[1]/pr[2]);       // final
    //
    //      undistorted[i] = fi;
    //    }

  }

  void PinholeCamera::projectPoints( const ObjectPointsVec &objectPoints, 
      const Vec3d &rvec, const Vec3d &tvec, ImagePointsVec &imagePoints, 
      OutputArray jacobian) const
  {
    // will support only 3-channel data now for points
    imagePoints.resize(objectPoints.size());

    //    Vec3d om( _rvec );
    //    Vec3d T( _tvec );
    //
    //    //CV_Assert(_K.size() == Size(3,3) && (_K.type() == CV_32F || _K.type() == CV_64F) && _D.type() == _K.type() && _D.total() == 4);
    //
    //    Matx33d K( matx() );
    //    Vec2d f( _fx, _fy );
    //    Vec2d c(K(0, 2), K(1, 2));

    //Vec4d k( _distCoeffs );

    // Need to be sure JacobianRow isn't being padded
    //    assert( sizeof(JacobianRow ) == 15 * sizeof(double) );
    //    JacobianRow *Jn = 0;
    //    if (jacobian.needed())
    //    {
    //      int nvars = 2 + 2 + 1 + 4 + 3 + 3; // f, c, alpha, k, om, T,
    //      jacobian.create(2*(int)objectPoints.size(), nvars, CV_64F);
    //      Jn = jacobian.getMat().ptr<JacobianRow>(0);
    //    }

    //Matx33d R;
    //Matx<double, 3, 9> dRdom;
    //Rodrigues(rvec, R, dRdom);
    Affine3d aff(rvec, tvec);

    //const Vec3f* Xf = objectPoints.getMat().ptr<Vec3f>();
    //const Vec3d* Xd = objectPoints.getMat().ptr<Vec3d>();
    //Vec2f *xpf = imagePoints.getMat().ptr<Vec2f>();
    //Vec2d *xpd = imagePoints.getMat().ptr<Vec2d>();

    for(size_t i = 0; i < objectPoints.size(); ++i)
    {
      Vec3d Xworld( objectPoints[i] );
      Vec3d Xcam( aff*Xworld );

      imagePoints[i] = image( warp( Xcam ) );

      //      if (jacobian.needed())
      //      {
      //        //Vec3d Xi = pdepth == CV_32F ? (Vec3d)Xf[i] : Xd[i];
      //        //Vec3d Y = aff*Xi;
      //        double dYdR[] = { Xi[0], Xi[1], Xi[2], 0, 0, 0, 0, 0, 0,
      //          0, 0, 0, Xi[0], Xi[1], Xi[2], 0, 0, 0,
      //          0, 0, 0, 0, 0, 0, Xi[0], Xi[1], Xi[2] };
      //
      //        Matx33d dYdom_data = Matx<double, 3, 9>(dYdR) * dRdom.t();
      //        const Vec3d *dYdom = (Vec3d*)dYdom_data.val;
      //
      //        Matx33d dYdT_data = Matx33d::eye();
      //        const Vec3d *dYdT = (Vec3d*)dYdT_data.val;
      //
      //        //Vec2d x(Y[0]/Y[2], Y[1]/Y[2]);
      //        Vec3d dxdom[2];
      //        dxdom[0] = (1.0/Y[2]) * dYdom[0] - x[0]/Y[2] * dYdom[2];
      //        dxdom[1] = (1.0/Y[2]) * dYdom[1] - x[1]/Y[2] * dYdom[2];
      //
      //        Vec3d dxdT[2];
      //        dxdT[0]  = (1.0/Y[2]) * dYdT[0] - x[0]/Y[2] * dYdT[2];
      //        dxdT[1]  = (1.0/Y[2]) * dYdT[1] - x[1]/Y[2] * dYdT[2];
      //
      //        //double r2 = x.dot(x);
      //        Vec3d dr2dom = 2 * x[0] * dxdom[0] + 2 * x[1] * dxdom[1];
      //        Vec3d dr2dT  = 2 * x[0] *  dxdT[0] + 2 * x[1] *  dxdT[1];
      //
      //        //double r = std::sqrt(r2);
      //        double drdr2 = r > 1e-8 ? 1.0/(2*r) : 1;
      //        Vec3d drdom = drdr2 * dr2dom;
      //        Vec3d drdT  = drdr2 * dr2dT;
      //
      //        // Angle of the incoming ray:
      //        //double theta = atan(r);
      //        double dthetadr = 1.0/(1+r2);
      //        Vec3d dthetadom = dthetadr * drdom;
      //        Vec3d dthetadT  = dthetadr *  drdT;
      //
      //        //double theta_d = theta + k[0]*theta3 + k[1]*theta5 + k[2]*theta7 + k[3]*theta9;
      //        double dtheta_ddtheta = 1 + 3*k[0]*theta2 + 5*k[1]*theta4 + 7*k[2]*theta6 + 9*k[3]*theta8;
      //        Vec3d dtheta_ddom = dtheta_ddtheta * dthetadom;
      //        Vec3d dtheta_ddT  = dtheta_ddtheta * dthetadT;
      //        Vec4d dtheta_ddk  = Vec4d(theta3, theta5, theta7, theta9);
      //
      //        //double inv_r = r > 1e-8 ? 1.0/r : 1;
      //        //double cdist = r > 1e-8 ? theta_d / r : 1;
      //        Vec3d dcdistdom = inv_r * (dtheta_ddom - cdist*drdom);
      //        Vec3d dcdistdT  = inv_r * (dtheta_ddT  - cdist*drdT);
      //        Vec4d dcdistdk  = inv_r *  dtheta_ddk;
      //
      //        //Vec2d xd1 = x * cdist;
      //        Vec4d dxd1dk[2];
      //        Vec3d dxd1dom[2], dxd1dT[2];
      //        dxd1dom[0] = x[0] * dcdistdom + cdist * dxdom[0];
      //        dxd1dom[1] = x[1] * dcdistdom + cdist * dxdom[1];
      //        dxd1dT[0]  = x[0] * dcdistdT  + cdist * dxdT[0];
      //        dxd1dT[1]  = x[1] * dcdistdT  + cdist * dxdT[1];
      //        dxd1dk[0]  = x[0] * dcdistdk;
      //        dxd1dk[1]  = x[1] * dcdistdk;
      //
      //        //Vec2d xd3(xd1[0] + alpha*xd1[1], xd1[1]);
      //        Vec4d dxd3dk[2];
      //        Vec3d dxd3dom[2], dxd3dT[2];
      //        dxd3dom[0] = dxd1dom[0] + _alpha * dxd1dom[1];
      //        dxd3dom[1] = dxd1dom[1];
      //        dxd3dT[0]  = dxd1dT[0]  + _alpha * dxd1dT[1];
      //        dxd3dT[1]  = dxd1dT[1];
      //        dxd3dk[0]  = dxd1dk[0]  + _alpha * dxd1dk[1];
      //        dxd3dk[1]  = dxd1dk[1];
      //
      //        Vec2d dxd3dalpha(xd1[1], 0);
      //
      //        //final jacobian
      //        Jn[0].dom = f[0] * dxd3dom[0];
      //        Jn[1].dom = f[1] * dxd3dom[1];
      //
      //        Jn[0].dT = f[0] * dxd3dT[0];
      //        Jn[1].dT = f[1] * dxd3dT[1];
      //
      //        Jn[0].dk = f[0] * dxd3dk[0];
      //        Jn[1].dk = f[1] * dxd3dk[1];
      //
      //        Jn[0].dalpha = f[0] * dxd3dalpha[0];
      //        Jn[1].dalpha = 0; //f[1] * dxd3dalpha[1];
      //
      //        Jn[0].df = Vec2d(xd3[0], 0);
      //        Jn[1].df = Vec2d(0, xd3[1]);
      //
      //        Jn[0].dc = Vec2d(1, 0);
      //        Jn[1].dc = Vec2d(0, 1);
      //
      //        //step to jacobian rows for next point
      //        Jn += 2;
      //      }
    }
  }


  ImagePointsVec PinholeCamera::image( const ImagePointsVec &vec ) const
  {
    ImagePointsVec out( vec.size() );
    std::transform( vec.begin(), vec.end(), out.begin(), makeImager() );
    return out; 
  }

  ImagePointsVec PinholeCamera::normalize( const ImagePointsVec &vec ) const
  {
    ImagePointsVec out;
    std::transform( vec.begin(), vec.end(), back_inserter( out ), makeNormalizer() );
    return out; 
  }

  ImagePointsVec PinholeCamera::unwarp( const ImagePointsVec &pw ) const
  {
    ImagePointsVec out( pw.size() );
    std::transform( pw.begin(), pw.end(), out.begin(), TxUndistorter( *this ) );
    return out; 
  }




};

