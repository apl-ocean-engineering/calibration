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



GraphCut::GraphCut( double colorWeight )
:   _colorWeight( colorWeight ), _image(), _mask(),
    _ignoreGMM(1), _bgdGMM( 5 ), _fgdGMM( 5 )
{;}

void GraphCut::setMask( const Mat &msk )
{
  msk.copyTo( _mask );

  CV_Assert( !_image.empty() );
  checkMask();
  initGMMs();
}

void GraphCut::setMaskRect( const Rect &r )
{
  CV_Assert( !_image.empty() );

  const Size imgSize( _image.size() );
  _mask.create( imgSize, CV_8UC1 );
  _mask.setTo( G_BGD );

  Rect rect(r);
  rect.x = std::max(0, rect.x);
  rect.y = std::max(0, rect.y);
  rect.width = std::min(rect.width, imgSize.width-rect.x);
  rect.height = std::min(rect.height, imgSize.height-rect.y);

  (_mask(rect)).setTo( Scalar(G_PR_FGD) );

  checkMask();
  initGMMs();
}

void GraphCut::setImage( const Mat &img )
{
  _image = img;

  if( false ) {
    cvtColor( _image, _csImage, cv::COLOR_BGR2Lab );
    LOG(INFO) << "Converting image to CIE Lab space.";
  } else {
    LOG(INFO) << "Using image in RGB space.";
    _image.copyTo( _csImage );
  }
}


void GraphCut::showMaxQImages( )
{
  CV_Assert( !_image.empty() && !_mask.empty() );

  // Assume the BG mask is more accurate than the FG mask (FG mask might
  // include some BG).  Update the mask using solely the BG GMM

  Mat bgdQimage( Mat::zeros( _image.size(), CV_8UC3 ) );
  Mat fgdQimage( Mat::zeros( _image.size(), CV_8UC3 ) );

  Spectrum bgdSpectrum( _bgdGMM.componentsCount() );
  Spectrum fgdSpectrum( _fgdGMM.componentsCount() );

  Point p;
  for( p.y = 0; p.y < _image.rows; p.y++ )
    for( p.x = 0; p.x < _image.cols; p.x++ )
    {
      int at;
      float q;
      Vec3d color = _csImage.at<Vec3b>(p);
      //if( _mask.at<uchar>(p) != G_PR_FGD ) continue;

      q = _fgdGMM.maxQat(color, at);
      fgdQimage.at<Vec3b>(p) = fgdSpectrum( at, q );

      q = _bgdGMM.maxQat(color, at);
      bgdQimage.at<Vec3b>(p) = bgdSpectrum( at, q );
    }

    imshow( "qimage fgb", fgdQimage );
    imshow( "qimage bgb", bgdQimage );
  }


void GraphCut::bgRefineMask( float pLimit )
{
  CV_Assert( !_image.empty() && !_mask.empty() );

  // Assume the BG mask is more accurate than the FG mask (FG mask might
  // include some BG).  Update the mask by identifying any PR_FGD points
  // which are matched by any bgd GMM with at least pLimit.
  // Switched those points to G_PR_BGD

  Point p;
  for( p.y = 0; p.y < _image.rows; p.y++ )
    for( p.x = 0; p.x < _image.cols; p.x++ )
    {
      Vec3d color = _csImage.at<Vec3b>(p);
      if( maskAt(p) != G_PR_FGD ) continue;

      float prob = _bgdGMM.maxQ(color);
      //LOG(INFO) << p << " : " << prob;

      if( prob > pLimit ) maskSet(p, G_PR_BGD);

    }

  // Relearn the GMMs based
  initGMMs();
}

void GraphCut::reassignFGtoBG( float pLimit )
{
  Point p;

  for( int ci = 0; ci < _fgdGMM.componentsCount(); ++ci ) {
    int idx;
    float prob = _bgdGMM.maxQat( _fgdGMM[ci].mean, idx );

    LOG(INFO) << "Fgd " << ci << " : " << _fgdGMM[ci].mean;

    for( int bci = 0; bci < _bgdGMM.componentsCount(); ++bci )
      LOG(INFO)<< "Bgd " << bci << " : " << _bgdGMM[bci].mean;


    LOG(INFO) << "Mean of fgd GMM " << ci << " has max Q = " << prob << " in bgd GMM " << idx;

    if( prob > pLimit ) {
      LOG(INFO) << "Reassigning FGB " << ci << " to background.";

      for( p.y = 0; p.y < _image.rows; p.y++ )
        for( p.x = 0; p.x < _image.cols; p.x++ )
          if( (_fgdGMM.whichComponent( _csImage.at<Vec3b>(p) ) == ci) && (maskAt(p) == G_PR_FGD) )  maskSet(p, G_PR_BGD );

    }
  }

  // Relearn the GMMs based
  initGMMs();
}


bool GraphCut::process( int iterCount )
{
  CV_Assert( !_image.empty() && !_mask.empty() );

  if( iterCount <= 0) return false;

  checkMask( );

  //const double gamma = 100; //50;

  NeighborWeights w;
  calcNeighborWeights( _csImage, w );

  Mat compIdxs( _image.size(), CV_32SC1 );

  for( int i = 0; i < iterCount; i++ )
  {
    GCGraph<double> graph;
    assignGMMsComponents( compIdxs );
    learnGMMs( compIdxs );
    constructGCGraph( w, graph );
    estimateSegmentation( graph );
  }

return true;
}


Mat GraphCut::drawMask( void ) const
{
  Mat image( Mat::zeros( _mask.size(), CV_8UC3 ) );

  const Scalar Red( 0, 0, 255 ), Yellow(0, 255, 255),
               Green(0, 255, 0), Blue(255,0,0),
               Grey( 128,128,128 );

  // Tried to do this with masks and bitwise operations.
  // image.setTo( Red, mask & G_FGD );
  // image.setTo( Yellow, mask & G_PR_FGD );
  // image.setTo( Green, mask & G_PR_BGD );
  // image.setTo( Blue, mask & G_BGD );
  // The G_* flags aren't bitwise, so that didn't work.

  // could do something like
  Mat bitmask;
  cv::compare( _mask, Scalar( G_FGD ), bitmask, CMP_EQ );
  image.setTo( Red, bitmask );

  cv::compare( _mask, Scalar( G_PR_FGD ), bitmask, CMP_EQ );
  image.setTo( Yellow, bitmask );

  cv::compare( _mask, Scalar( G_PR_BGD ), bitmask, CMP_EQ );
  image.setTo( Green, bitmask );

  cv::compare( _mask, Scalar( G_BGD ), bitmask, CMP_EQ );
  image.setTo( Blue, bitmask );

  cv::compare( _mask, Scalar( G_IGNORE ), bitmask, CMP_EQ );
  image.setTo( Grey, bitmask );


return image;
}


static unsigned char countBits( unsigned char v )
{
  unsigned char c; // store the total here
  static const int S[] = {1, 2, 4}; // Magic Binary Numbers
  static const int B[] = {0x55, 0x33, 0x0F};

  c = v - ((v >> 1) & B[0]);
  c = ((c >> S[1]) & B[1]) + (c & B[1]);
  c = ((c >> S[2]) + c) & B[2];

  return c;
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

  Point p;
  for(  p.y = 0; p.y < _mask.rows; p.y++ )
    for(  p.x = 0; p.x < _mask.cols; p.x++ )
    {
      uchar val = maskAt(p);
      if( countBits(val) > 1 )
        CV_Error( CV_StsBadArg, "_mask element value must a single GraphCutLabels value" );
    }
}


/*
Initialize GMM background and foreground models using kmeans algorithm.
*/
bool GraphCut::initGMMs( void )
{
  const int kMeansAttempts = 1;
  const int kMeansIterations = 100;
  const int kMeansType = KMEANS_PP_CENTERS;; //KMEANS_RANDOM_CENTERS; //

  std::vector<Vec3f> bgdSamples, fgdSamples, ignoreSamples;

  Point p;
  for( p.y = 0; p.y < _csImage.rows; p.y++ )
    for( p.x = 0; p.x < _csImage.cols; p.x++ )
    {

      Vec3f color = (Vec3f)_csImage.at<Vec3b>(p);

      if( maskAt(p) == G_BGD || maskAt(p) == G_PR_BGD )
        bgdSamples.push_back( color );
      else if( maskAt(p) == G_BGD || maskAt(p) == G_PR_BGD )// G_FGD | G_PR_FGD
        fgdSamples.push_back( color );
      else if( maskAt(p) == G_IGNORE )
        ignoreSamples.push_back( color );

    }


  if( bgdSamples.empty() || fgdSamples.empty() ) { return false; }

  Mat bgdLabels, fgdLabels;
  //Mat _bgdSamples( (int)bgdSamples.size(), 3, CV_32FC1, &bgdSamples[0][0] );

  kmeans( bgdSamples, _bgdGMM.componentsCount(), bgdLabels,
          TermCriteria( CV_TERMCRIT_ITER, kMeansIterations, 1e-6), kMeansAttempts, kMeansType );

  // Mat _fgdSamples( (int)fgdSamples.size(), 3, CV_32FC1, &fgdSamples[0][0] );
  kmeans( fgdSamples, _fgdGMM.componentsCount(), fgdLabels,
          TermCriteria( CV_TERMCRIT_ITER, kMeansIterations, 1e-6), kMeansAttempts, kMeansType );

  _bgdGMM.initLearning();
  for( int i = 0; i < (int)bgdSamples.size(); i++ ) _bgdGMM.addSample( bgdLabels.at<int>(i,0), bgdSamples[i] );
  _bgdGMM.endLearning();

  _fgdGMM.initLearning();
  for( int i = 0; i < (int)fgdSamples.size(); i++ ) _fgdGMM.addSample( fgdLabels.at<int>(i,0), fgdSamples[i] );
  _fgdGMM.endLearning();

  _ignoreGMM.initLearning();
  for( int i = 0; i < (int)ignoreSamples.size(); i++ ) _ignoreGMM.addSample( 0, ignoreSamples[i] );
  _ignoreGMM.endLearning();

return true;
}

/*
Assign GMMs components for each pixel.
*/
void GraphCut::assignGMMsComponents( Mat& compIdxs )
{
  Point p;
  for( p.y = 0; p.y < _csImage.rows; p.y++ )
    for( p.x = 0; p.x < _csImage.cols; p.x++ )
    {
      Vec3d color = _csImage.at<Vec3b>(p);

      if (_mask.at<uchar>(p) == G_BGD || _mask.at<uchar>(p) == G_PR_BGD)
        compIdxs.at<int>(p) = _bgdGMM.whichComponent(color);
      else
        compIdxs.at<int>(p) = _fgdGMM.whichComponent(color);
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
      for( p.x = 0; p.x < _image.cols; p.x++ )
      {
        int ci = compIdxs.at<int>(p);
        // if( compIdxs.at<int>(p) == ci )
        // {
        if( maskAt(p) == G_BGD || maskAt(p) == G_PR_BGD ) {
          _bgdGMM.addSample( ci, _image.at<Vec3b>(p) );
        } else {
          _fgdGMM.addSample( ci, _image.at<Vec3b>(p) );
        }
        //}

    }
//  }
  _bgdGMM.endLearning();
  _fgdGMM.endLearning();
}

/*
Construct GCGraph
*/
void GraphCut::constructGCGraph( const NeighborWeights &w, GCGraph<double>& graph )
{
  const double lambda = 9*_colorWeight;

    int vtxCount = _image.cols*_image.rows,
    edgeCount = 2*(4*_image.cols*_image.rows - 3*(_image.cols + _image.rows) + 2);
    graph.create(vtxCount, edgeCount);

    Point p;
    for( p.y = 0; p.y < _image.rows; p.y++ )
      for( p.x = 0; p.x < _image.cols; p.x++)
      {
        // add node
        int vtxIdx = graph.addVtx();
        Vec3b color = _csImage.at<Vec3b>(p);

        // set t-weights
        double fromSource, toSink;
        if( maskAt(p) == G_PR_BGD || maskAt(p) == G_PR_FGD )
        {
          fromSource = -_bgdGMM.logLikelihood(color);
          toSink =     -_fgdGMM.logLikelihood(color);
        }
        else if( maskAt(p) == G_BGD )
        {
          fromSource = 0;
          toSink = lambda;
        }
        else // G_FGD
        {
          fromSource = lambda;
          toSink = 0;
        }
        graph.addTermWeights( vtxIdx, fromSource, toSink );


        // set n-weights
        if( p.x>0 )
        {
          double wt = w.left.at<double>(p);
          graph.addEdges( vtxIdx, vtxIdx-1, wt, wt );
        }
        if( p.x>0 && p.y>0 )
        {
          double wt = w.upleft.at<double>(p);
          graph.addEdges( vtxIdx, vtxIdx-_image.cols-1, wt, wt );
        }
        if( p.y>0 )
        {
          double wt = w.up.at<double>(p);
          graph.addEdges( vtxIdx, vtxIdx-_image.cols, wt, wt );
        }
        if( p.x<_image.cols-1 && p.y>0 )
        {
          double wt = w.upright.at<double>(p);
          graph.addEdges( vtxIdx, vtxIdx-_image.cols+1, wt, wt );
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
      for( p.x = 0; p.x < _mask.cols; p.x++ )
      {
        if( maskAt(p) == G_PR_BGD || maskAt(p) == G_PR_FGD )
        {
          if( graph.inSourceSegment( p.y*_mask.cols+p.x /*vertex index*/ ) )
            maskSet(p, G_PR_FGD );
          else
            maskSet(p, G_PR_BGD );
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
  void GraphCut::calcNeighborWeights( const Mat& img, NeighborWeights &w )
  {
    const double beta = calcBeta( _image );


    const double gamma = _colorWeight,
                gammaDivSqrt2 = _colorWeight / std::sqrt(2.0f);      // Reduced weighting for diagonal terms

    w.left.create(   img.size(), CV_64FC1 );
    w.upleft.create( img.size(), CV_64FC1 );
    w.up.create(     img.size(), CV_64FC1 );
    w.upright.create( img.size(), CV_64FC1 );

    Point p;
    for( p.y = 0; p.y < img.rows; p.y++ )
      for( p.x = 0; p.x < img.cols; p.x++ )
      {
        Vec3d color = img.at<Vec3b>(p);

        if( p.x-1>=0 ) // left
        {
          Vec3d diff = color - (Vec3d)img.at<Vec3b>(p.y,p.x-1);
          w.left.at<double>(p) = gamma * exp(-beta*diff.dot(diff));
        }
        else
          w.left.at<double>(p) = 0;

        if( p.x-1>=0 && p.y-1>=0 ) // upleft
        {
          Vec3d diff = color - (Vec3d)img.at<Vec3b>(p.y-1,p.x-1);
          w.upleft.at<double>(p) = gammaDivSqrt2 * exp(-beta*diff.dot(diff));
        }
        else
          w.upleft.at<double>(p) = 0;

        if( p.y-1>=0 ) // up
        {
          Vec3d diff = color - (Vec3d)img.at<Vec3b>(p.y-1,p.x);
          w.up.at<double>(p) = gamma * exp(-beta*diff.dot(diff));
        }
        else
          w.up.at<double>(p) = 0;

        if( p.x+1 < img.cols && p.y-1 >= 0 ) // upright
        {
          Vec3d diff = color - (Vec3d)img.at<Vec3b>(p.y-1,p.x+1);
          w.upright.at<double>(p) = gammaDivSqrt2 * exp(-beta*diff.dot(diff));
        }
        else
          w.upright.at<double>(p) = 0;
      }

  }
