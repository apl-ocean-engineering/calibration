
#ifndef __COMPOSITE_IMAGE_H__
#define __COMPOSITE_IMAGE_H__

#include <string>

#include <opencv2/core/core.hpp>

#include "image_pair.h"

namespace AplCam {

  using cv::Mat;
  using cv::Rect;
  using cv::Size;

  struct CompositeCanvas
  {
    CompositeCanvas( void ) {;}

    CompositeCanvas( const Size &sz, int type )
      : canvas( sz, type )
    {
rect[0] = Rect( 0,0, sz.width/2, sz.height );
rect[1] = Rect( rect[0].width, 0, rect[0].width, rect[0].height );

      roi[0] = Mat( canvas, rect[0] );
      roi[1] = Mat( canvas, rect[1] );
    }

    CompositeCanvas( const Mat &mat, const Rect &roi1 = Rect(), const Rect &roi2 = Rect() )
      : canvas( mat )
    {
      if( roi1.area() > 0 ) {
        rect[0] = roi1;
      } else {
        rect[0] = Rect( 0,0, mat.size().width / 2, mat.size().height );
      }

      if( roi2.area() > 0 ) {
        rect[1] = roi2;
      } else {
        rect[1] = Rect( rect[0].width, 0, mat.size().width - rect[0].width, mat.size().height );
      }

      roi[0] = Mat( canvas, rect[0] );
      roi[1] = Mat( canvas, rect[1] );
    }

    CompositeCanvas( const Mat &mat0, const Mat &mat1, bool doCopy = true )
      : canvas()
    {
      Size canvasSize( mat0.size().width + mat1.size().width, 
          std::max(mat0.size().height, mat1.size().height) ); 

      canvas.create( canvasSize, mat0.type() );

      rect[0] = Rect( 0,0, mat0.size().width, mat0.size().height );
      rect[1] = Rect( mat0.size().width ,0, mat1.size().width, mat1.size().height );

      roi[0] = Mat( canvas, rect[0] );
      roi[1] = Mat( canvas, rect[1] );

      if( doCopy ) {
      mat0.copyTo( roi[0] );
      mat1.copyTo( roi[1] );
      }
    }


//    CompositeCanvas( const ImagePair &pair )
//      : canvas()
//    {
//      // I really should do this with undistorted images...
//      canvas.create( std::max( pair[0].size().height, pair[1].size().height ),
//          pair[0].size().width + pair[1].size().width,
//          pair[0].img().type() );
//
//      rect[0] = Rect( 0, 0, pair[0].size().width, pair[0].size().height );
//      rect[1] = Rect( rect[0].width, 0, pair[1].size().width, pair[1].size().height );
//      roi[0] = Mat( canvas, rect[0] );
//      roi[1] = Mat( canvas, rect[1] );
//    }

    operator Mat &() { return canvas; }
    operator cv::_InputArray() { return cv::_InputArray(canvas); }

    Mat &operator[]( int i ){ return roi[i]; }
    const Mat &operator[]( int i ) const { return roi[i]; }

    Size size( void ) const { return canvas.size(); }

    Mat scaled( float scale ) const {
      Mat out;
      resize( canvas, out, Size(), scale, scale, cv::INTER_LINEAR );
      return out;
    }

    cv::Point origin( int i ) 
    { return cv::Point( rect[i].x, rect[i].y ); }


    Mat canvas, roi[2];
    Rect rect[2];
  };

  struct CompositeVideo
  { 
    CompositeVideo( const std::string &filepath )
      : _filepath( filepath ), _video( filepath ) {;}

    bool isOpened( void ) const { return _video.isOpened(); }

    bool read(  CompositeCanvas &canvas )
    { 
      Mat frame;
      if( _video.read( frame ) ) {
        canvas = CompositeCanvas( frame );
        return true;
      }
      return false;
    }

    bool seek( double frame ) { return _video.set( CV_CAP_PROP_POS_FRAMES, frame ); }
    bool rewind( void )       { return seek(0); }
      

    std::string _filepath;
    cv::VideoCapture _video;
  };




}

#endif

