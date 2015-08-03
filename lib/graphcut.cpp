/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
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
//   * The name of Intel Corporation may not be used to endorse or promote products
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

#include <glog/logging.h>

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <limits>

#include "spectrum.h"
#include "graphcut.h"

using namespace cv;

/*
This is implementation of image segmentation algorithm GrabCut described in
"GrabCut — Interactive Foreground Extraction using Iterated Graph Cuts".
Carsten Rother, Vladimir Kolmogorov, Andrew Blake.
*/


/*
Initialize mask using rectangular.
*/
static void initMaskWithRect( Mat& mask, Size _imageSize, Rect rect )
{
  mask.create( _imageSize, CV_8UC1 );
  mask.setTo( GC_BGD );

  rect.x = std::max(0, rect.x);
  rect.y = std::max(0, rect.y);
  rect.width = std::min(rect.width, _imageSize.width-rect.x);
  rect.height = std::min(rect.height, _imageSize.height-rect.y);

  (mask(rect)).setTo( Scalar(GC_PR_FGD) );
}

//   void graphCut( InputArray _image, InputOutputArray _mask, Rect rect,
//     InputOutputArray _bgdModel, InputOutputArray _fgdModel,
//     int iterCount, int mode )
//     {
//       Mat _image = _image.getMat();
//       Mat& mask = _mask.getMatRef();
//       Mat& bgdModel = _bgdModel.getMatRef();
//       Mat& fgdModel = _fgdModel.getMatRef();
//
//       if( _image.empty() )
//       CV_Error( CV_StsBadArg, "image is empty" );
//       if( _image.type() != CV_8UC3 )
//       CV_Error( CV_StsBadArg, "image mush have CV_8UC3 type" );
//
//       GMM bgdGMM( bgdModel ), fgdGMM( fgdModel );
//       Mat compIdxs( _image.size(), CV_32SC1 );
//
//       if( iterCount <= 0)
//         return;
//
//       if( mode == GC_INIT_WITH_RECT || mode == GC_INIT_WITH_MASK )
//       {
//         if( mode == GC_INIT_WITH_RECT )
//         initMaskWithRect( mask, _image.size(), rect );
//         else // flag == GC_INIT_WITH_MASK
//         checkMask( _image, mask );
//         initGMMs( _image, mask, bgdGMM, fgdGMM );
//       } else if( mode == GC_EVAL ) {
//         checkMask( _image, mask );
// }
//
//       const double gamma = 50;
//       const double lambda = 9*gamma;
//       const double beta = calcBeta( _image );
//
//       Mat leftW, upleftW, upW, uprightW;
//       calcNWeights( _image, leftW, upleftW, upW, uprightW, beta, gamma );
//
//       for( int i = 0; i < iterCount; i++ )
//       {
//         GCGraph<double> graph;
//         assignGMMsComponents( _image, mask, bgdGMM, fgdGMM, compIdxs );
//         learnGMMs( _image, mask, compIdxs, bgdGMM, fgdGMM );
//         constructGCGraph(_image, mask, bgdGMM, fgdGMM, lambda, leftW, upleftW, upW, uprightW, graph );
//         estimateSegmentation( graph, mask );
//       }
//     }


GraphCut::GraphCut( void )
:   _image(), _mask(), _bgdGMM(), _fgdGMM()
{;}

void GraphCut::setMask( const Mat &msk )
{
  msk.copyTo( _mask );

  CV_Assert( !_image.empty() );
  checkMask();
  initGMMs();
}

void GraphCut::setMaskRect( const Rect &rect )
{
  CV_Assert( !_image.empty() );
  initMaskWithRect( _mask, _image.size(), rect );

  checkMask();
  initGMMs();
}

void GraphCut::setImage( const Mat &img )
{
  _image = img;

cvtColor( _image, _csImage, cv::COLOR_BGR2Lab );
//_image.copyTo( _csImage );
}


void GraphCut::showMaxQImages( )
{
  CV_Assert( !_image.empty() && !_mask.empty() );

  // Assume the BG mask is more accurate than the FG mask (FG mask might
  // include some BG).  Update the mask using solely the BG GMM

  Mat bgdQimage( Mat::zeros( _image.size(), CV_8UC3 ) );
  Mat fgdQimage( Mat::zeros( _image.size(), CV_8UC3 ) );

Spectrum bgdSpectrum( _bgdGMM.componentsCount );
Spectrum fgdSpectrum( _fgdGMM.componentsCount );


  Point p;
  for( p.y = 0; p.y < _image.rows; p.y++ )
  for( p.x = 0; p.x < _image.cols; p.x++ )
  {
int at;
float q;
    Vec3d color = _csImage.at<Vec3b>(p);
    //if( _mask.at<uchar>(p) != GC_PR_FGD ) continue;

    q = _fgdGMM.maxQat(color, at);
    fgdQimage.at<Vec3b>(p) = fgdSpectrum( at, q );

    q = _bgdGMM.maxQat(color, at);
    bgdQimage.at<Vec3b>(p) = bgdSpectrum( at, q );
  }

  imshow( "qimage fgb", fgdQimage );
  imshow( "qimage bgb", bgdQimage );

  // Relearn the GMMs based
  initGMMs();
}


void GraphCut::bgRefineMask( float pLimit )
{
  CV_Assert( !_image.empty() && !_mask.empty() );

  // Assume the BG mask is more accurate than the FG mask (FG mask might
  // include some BG).  Update the mask using solely the BG GMM

  Point p;
  for( p.y = 0; p.y < _image.rows; p.y++ )
  for( p.x = 0; p.x < _image.cols; p.x++ )
  {
    Vec3d color = _csImage.at<Vec3b>(p);
    if( _mask.at<uchar>(p) != GC_PR_FGD ) continue;

    float prob = _bgdGMM.maxQ(color);
    //LOG(INFO) << p << " : " << prob;

    if( prob > pLimit ) _mask.at<uchar>(p) = GC_PR_BGD;

  }

  // Relearn the GMMs based
  initGMMs();

  for( int ci = 0; ci < _fgdGMM.componentsCount; ++ci ) {
    int idx;
    float prob = _bgdGMM.maxQat( _fgdGMM.mean( ci ), idx );

LOG(INFO) << "Fgd " << ci << " : " << _fgdGMM.mean( ci );
  for( int bci = 0; bci < _bgdGMM.componentsCount; ++bci )
    LOG(INFO)<< "Bgd " << bci << " : " << _bgdGMM.mean( bci );


    LOG(INFO) << "mean of fgd GMM " << ci << " has max Q=" << prob << " in bgd GMM " << idx;

    if( prob > 1e-4 ) {
LOG(INFO) << "Reassigning FGB " << ci << " to background.";

      for( p.y = 0; p.y < _image.rows; p.y++ )
      for( p.x = 0; p.x < _image.cols; p.x++ )
      if( (_fgdGMM.whichComponent( _csImage.at<Vec3b>(p) ) == ci) && (_mask.at<uchar>(p) = GC_PR_FGD) )
        _mask.at<uchar>(p) = GC_PR_BGD;

    }
  }

  // Relearn the GMMs based
  initGMMs();
}


void GraphCut::process( int iterCount )
{
  CV_Assert( !_image.empty() && !_mask.empty() );

  // if( _image.type() != CV_8UC3 )
  // CV_Error( CV_StsBadArg, "image mush have CV_8UC3 type" );

  Mat compIdxs( _image.size(), CV_32SC1 );

  if( iterCount <= 0) return;

  // if( mode == GC_INIT_WITH_RECT || mode == GC_INIT_WITH_MASK )
  // {
  //   if( mode == GC_INIT_WITH_RECT )
  //   initMaskWithRect( mask, _image.size(), rect );
  //   else // flag == GC_INIT_WITH_MASK
  //   checkMask( _image, mask );
  //   initGMMs( _image, mask, bgdGMM, fgdGMM );
  // } else if( mode == GC_EVAL ) {
  checkMask( );
  //    }

  const double gamma = 50;
  const double lambda = 9*gamma;
  const double beta = calcBeta( _image );

  Mat leftW, upleftW, upW, uprightW;
  calcNWeights( _csImage, leftW, upleftW, upW, uprightW, beta, gamma );

  for( int i = 0; i < iterCount; i++ )
  {
    GCGraph<double> graph;
    assignGMMsComponents( compIdxs );
    learnGMMs( compIdxs );
    constructGCGraph( lambda, leftW, upleftW, upW, uprightW, graph );
    estimateSegmentation( graph );
  }
}

Mat GraphCut::drawMask( void ) const
{
  Mat image( Mat::zeros( _mask.size(), CV_8UC3 ) );

  const Scalar Red( 0, 0, 255 ), Yellow(0, 255, 255),
  Green(0, 255, 0), Blue(255,0,0), Black(0,0,0);

  // Tried to do this with masks and bitwise operations.
  // image.setTo( Red, mask & GC_FGD );
  // image.setTo( Yellow, mask & GC_PR_FGD );
  // image.setTo( Green, mask & GC_PR_BGD );
  // image.setTo( Blue, mask & GC_BGD );

  // The GC_* flags are bitwise, so that didn't work.
  // could do something like
  Mat bitmask;
  cv::compare( _mask, Scalar( GC_FGD ), bitmask, CMP_EQ );
  image.setTo( Red, bitmask );

  cv::compare( _mask, Scalar( GC_PR_FGD ), bitmask, CMP_EQ );
  image.setTo( Yellow, bitmask );

  cv::compare( _mask, Scalar( GC_PR_BGD ), bitmask, CMP_EQ );
  image.setTo( Green, bitmask );

  cv::compare( _mask, Scalar( GC_BGD ), bitmask, CMP_EQ );
  image.setTo( Blue, bitmask );

return image;
}



//=== Protected functions =================

/*
Check size, type and element values of mask matrix.
*/
void GraphCut::checkMask( void )
{
  if( _mask.empty() )
  CV_Error( CV_StsBadArg, "_mask is empty" );
  if( _mask.type() != CV_8UC1 )
  CV_Error( CV_StsBadArg, "_mask must have CV_8UC1 type" );
  if( _mask.cols != _image.cols || _mask.rows != _image.rows )
  CV_Error( CV_StsBadArg, "_mask must have as many rows and cols as _image" );
  for( int y = 0; y < _mask.rows; y++ )
  {
    for( int x = 0; x < _mask.cols; x++ )
    {
      uchar val = _mask.at<uchar>(y,x);
      if( val!=GC_BGD && val!=GC_FGD && val!=GC_PR_BGD && val!=GC_PR_FGD )
      CV_Error( CV_StsBadArg, "_mask element value must be GC_BGD or GC_FGD or GC_PR_BGD or GC_PR_FGD" );
    }
  }
}


/*
Initialize GMM background and foreground models using kmeans algorithm.
*/
void GraphCut::initGMMs( void )
{
const int kMeansAttempts = 1;
  const int kMeansIterations = 100;
  const int kMeansType = KMEANS_PP_CENTERS;; //KMEANS_RANDOM_CENTERS; //


  std::vector<Vec3f> bgdSamples, fgdSamples;
  std::vector<Vec3f> bgdScaledSamples, fgdScaledSamples;

  Point p;
  for( p.y = 0; p.y < _csImage.rows; p.y++ )
  {
    for( p.x = 0; p.x < _csImage.cols; p.x++ )
    {
Vec3f color = (Vec3f)_csImage.at<Vec3b>(p);


      if( _mask.at<uchar>(p) == GC_BGD || _mask.at<uchar>(p) == GC_PR_BGD ){
        bgdSamples.push_back( color );
//color[0] *= 0.1;
        bgdScaledSamples.push_back( color );
      }else {// GC_FGD | GC_PR_FGD
        fgdSamples.push_back( color );
        //color[0] *= 0.1;
                fgdScaledSamples.push_back( color );
}
    }
  }

  CV_Assert( !bgdSamples.empty() && !fgdSamples.empty() );

  Mat bgdLabels, fgdLabels;
  //Mat _bgdSamples( (int)bgdSamples.size(), 3, CV_32FC1, &bgdSamples[0][0] );

  kmeans( bgdScaledSamples, _bgdGMM.componentsCount, bgdLabels,
          TermCriteria( CV_TERMCRIT_ITER, kMeansIterations, 1e-6), kMeansAttempts, kMeansType );

  // Mat _fgdSamples( (int)fgdSamples.size(), 3, CV_32FC1, &fgdSamples[0][0] );
  kmeans( fgdScaledSamples, _fgdGMM.componentsCount, fgdLabels,
          TermCriteria( CV_TERMCRIT_ITER, kMeansIterations, 1e-6), kMeansAttempts, kMeansType );

  _bgdGMM.initLearning();
  for( int i = 0; i < (int)bgdSamples.size(); i++ ) _bgdGMM.addSample( bgdLabels.at<int>(i,0), bgdSamples[i] );
  _bgdGMM.endLearning();

  _fgdGMM.initLearning();
  for( int i = 0; i < (int)fgdSamples.size(); i++ ) _fgdGMM.addSample( fgdLabels.at<int>(i,0), fgdSamples[i] );
  _fgdGMM.endLearning();
}

/*
Assign GMMs components for each pixel.
*/
void GraphCut::assignGMMsComponents( Mat& compIdxs )
{
  Point p;
  for( p.y = 0; p.y < _csImage.rows; p.y++ )
  {
    for( p.x = 0; p.x < _csImage.cols; p.x++ )
    {
      Vec3d color = _csImage.at<Vec3b>(p);
      compIdxs.at<int>(p) = (_mask.at<uchar>(p) == GC_BGD || _mask.at<uchar>(p) == GC_PR_BGD) ?  _bgdGMM.whichComponent(color) : _fgdGMM.whichComponent(color);
    }
  }
}

/*
Learn GMMs parameters.
*/
void GraphCut::learnGMMs( const Mat& compIdxs )
{
  _bgdGMM.initLearning();
  _fgdGMM.initLearning();
  Point p;
//  for( int ci = 0; ci < GMM::componentsCount; ci++ )
//  {
    for( p.y = 0; p.y < _image.rows; p.y++ )
    {
      for( p.x = 0; p.x < _image.cols; p.x++ )
      {
        int ci = compIdxs.at<int>(p);
        // if( compIdxs.at<int>(p) == ci )
        // {
        if( _mask.at<uchar>(p) == GC_BGD || _mask.at<uchar>(p) == GC_PR_BGD ) {
          _bgdGMM.addSample( ci, _image.at<Vec3b>(p) );
        } else {
          _fgdGMM.addSample( ci, _image.at<Vec3b>(p) );
        }
        //}
      }
    }
//  }
  _bgdGMM.endLearning();
  _fgdGMM.endLearning();
}

/*
Construct GCGraph
*/
void GraphCut::constructGCGraph( double lambda,
  const Mat& leftW, const Mat& upleftW, const Mat& upW, const Mat& uprightW, GCGraph<double>& graph )
  {
    int vtxCount = _image.cols*_image.rows,
    edgeCount = 2*(4*_image.cols*_image.rows - 3*(_image.cols + _image.rows) + 2);
    graph.create(vtxCount, edgeCount);
    Point p;
    for( p.y = 0; p.y < _image.rows; p.y++ )
    {
      for( p.x = 0; p.x < _image.cols; p.x++)
      {
        // add node
        int vtxIdx = graph.addVtx();
        Vec3b color = _csImage.at<Vec3b>(p);

        // set t-weights
        double fromSource, toSink;
        if( _mask.at<uchar>(p) == GC_PR_BGD || _mask.at<uchar>(p) == GC_PR_FGD )
        {
          fromSource = -log( _bgdGMM(color) );
          toSink = -log( _fgdGMM(color) );
        }
        else if( _mask.at<uchar>(p) == GC_BGD )
        {
          fromSource = 0;
          toSink = lambda;
        }
        else // GC_FGD
        {
          fromSource = lambda;
          toSink = 0;
        }
        graph.addTermWeights( vtxIdx, fromSource, toSink );

        // set n-weights
        if( p.x>0 )
        {
          double w = leftW.at<double>(p);
          graph.addEdges( vtxIdx, vtxIdx-1, w, w );
        }
        if( p.x>0 && p.y>0 )
        {
          double w = upleftW.at<double>(p);
          graph.addEdges( vtxIdx, vtxIdx-_image.cols-1, w, w );
        }
        if( p.y>0 )
        {
          double w = upW.at<double>(p);
          graph.addEdges( vtxIdx, vtxIdx-_image.cols, w, w );
        }
        if( p.x<_image.cols-1 && p.y>0 )
        {
          double w = uprightW.at<double>(p);
          graph.addEdges( vtxIdx, vtxIdx-_image.cols+1, w, w );
        }
      }
    }
  }

  /*
  Estimate segmentation using MaxFlow algorithm
  */
  void GraphCut::estimateSegmentation( GCGraph<double>& graph )
  {
    graph.maxFlow();
    Point p;
    for( p.y = 0; p.y < _mask.rows; p.y++ )
    {
      for( p.x = 0; p.x < _mask.cols; p.x++ )
      {
        if( _mask.at<uchar>(p) == GC_PR_BGD || _mask.at<uchar>(p) == GC_PR_FGD )
        {
          if( graph.inSourceSegment( p.y*_mask.cols+p.x /*vertex index*/ ) )
          _mask.at<uchar>(p) = GC_PR_FGD;
          else
          _mask.at<uchar>(p) = GC_PR_BGD;
        }
      }
    }
  }


  /*
  Calculate beta - parameter of GrabCut algorithm.
  beta = 1/(2*avg(sqr(||color[i] - color[j]||)))
  */
  double GraphCut::calcBeta( const Mat& _image )
  {
    double beta = 0;
    for( int y = 0; y < _image.rows; y++ )
    {
      for( int x = 0; x < _image.cols; x++ )
      {
        Vec3d color = _csImage.at<Vec3b>(y,x);
        if( x>0 ) // left
        {
          Vec3d diff = color - (Vec3d)_csImage.at<Vec3b>(y,x-1);
          beta += diff.dot(diff);
        }
        if( y>0 && x>0 ) // upleft
        {
          Vec3d diff = color - (Vec3d)_csImage.at<Vec3b>(y-1,x-1);
          beta += diff.dot(diff);
        }
        if( y>0 ) // up
        {
          Vec3d diff = color - (Vec3d)_csImage.at<Vec3b>(y-1,x);
          beta += diff.dot(diff);
        }
        if( y>0 && x<_image.cols-1) // upright
        {
          Vec3d diff = color - (Vec3d)_csImage.at<Vec3b>(y-1,x+1);
          beta += diff.dot(diff);
        }
      }
    }
    if( beta <= std::numeric_limits<double>::epsilon() )
    beta = 0;
    else
    beta = 1.f / (2 * beta/(4*_image.cols*_image.rows - 3*_image.cols - 3*_image.rows + 2) );

    return beta;
  }

  /*
  Calculate weights of noterminal vertices of graph.
  beta and gamma - parameters of GrabCut algorithm.
  */
  void GraphCut::calcNWeights( const Mat& img, Mat& leftW, Mat& upleftW, Mat& upW, Mat& uprightW, double beta, double gamma )
  {
    const double gammaDivSqrt2 = gamma / std::sqrt(2.0f);
    leftW.create( img.rows, img.cols, CV_64FC1 );
    upleftW.create( img.rows, img.cols, CV_64FC1 );
    upW.create( img.rows, img.cols, CV_64FC1 );
    uprightW.create( img.rows, img.cols, CV_64FC1 );
    for( int y = 0; y < img.rows; y++ )
    {
      for( int x = 0; x < img.cols; x++ )
      {
        Vec3d color = img.at<Vec3b>(y,x);
        if( x-1>=0 ) // left
        {
          Vec3d diff = color - (Vec3d)img.at<Vec3b>(y,x-1);
          leftW.at<double>(y,x) = gamma * exp(-beta*diff.dot(diff));
        }
        else
        leftW.at<double>(y,x) = 0;
        if( x-1>=0 && y-1>=0 ) // upleft
        {
          Vec3d diff = color - (Vec3d)img.at<Vec3b>(y-1,x-1);
          upleftW.at<double>(y,x) = gammaDivSqrt2 * exp(-beta*diff.dot(diff));
        }
        else
        upleftW.at<double>(y,x) = 0;
        if( y-1>=0 ) // up
        {
          Vec3d diff = color - (Vec3d)img.at<Vec3b>(y-1,x);
          upW.at<double>(y,x) = gamma * exp(-beta*diff.dot(diff));
        }
        else
        upW.at<double>(y,x) = 0;
        if( x+1<img.cols && y-1>=0 ) // upright
        {
          Vec3d diff = color - (Vec3d)img.at<Vec3b>(y-1,x+1);
          uprightW.at<double>(y,x) = gammaDivSqrt2 * exp(-beta*diff.dot(diff));
        }
        else
        uprightW.at<double>(y,x) = 0;
      }
    }
  }
