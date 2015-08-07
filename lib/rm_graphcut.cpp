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
#include "rm_graphcut.h"

using namespace cv;



#include "count_bits.h"


RMGraphCut::RMGraphCut( double colorWeight )
:   _colorWeight( colorWeight ), _image(), _labels(),
_gmm(15)
{;}

void RMGraphCut::setLabels( const Mat &labels )
{
  labels.copyTo( _labels );

  CV_Assert( !_image.empty() );
  checkLabels();
  initGMMs();
}

unsigned int RMGraphCut::labelCount( LabelType mask )
{
  unsigned int count = 0;
  Point p;
  for( p.y = 0; p.y < _image.rows; p.y++ )
    for( p.x = 0; p.x < _image.cols; p.x++ )
      if( labelAt(p) & mask ) ++count;

  return count;
}

void RMGraphCut::setImage( const Mat &img )
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


void RMGraphCut::showMaxQImages( )
{
  CV_Assert( !_image.empty() && !_labels.empty() );

  Mat bgdQimage( Mat::zeros( _image.size(), CV_8UC3 ) );
  Mat fgdQimage( Mat::zeros( _image.size(), CV_8UC3 ) );
  Mat allQimage( Mat::zeros( _image.size(), CV_8UC3 ) );

  //Spectrum bgdSpectrum( _gmm.componentsCount( G_BGD_MASK ) );
  //Spectrum fgdSpectrum( _gmm.componentsCount( G_FGD_MASK ) );
  Spectrum allSpectrum( _gmm.componentsCount() );

  Point p;
  for( p.y = 0; p.y < _image.rows; p.y++ )
    for( p.x = 0; p.x < _image.cols; p.x++ )
    {
      int at;
      float q;
      Vec3d color = _csImage.at<Vec3b>(p);
      //if( _mask.at<uchar>(p) != G_PR_FGD ) continue;

      // q = _gmm.maxQat( G_FGD_MASK, color, at);
      // if(at >= 0) fgdQimage.at<Vec3b>(p) = fgdSpectrum( at, q );
      //
      // q = _gmm.maxQat( G_BGD_MASK, color, at);
      // if(at >= 0) bgdQimage.at<Vec3b>(p) = bgdSpectrum( at, q );

      q = _gmm.maxQat( color, at );
      allQimage.at<Vec3b>(p) = allSpectrum( at, q );

      if( _gmm.maskAt( at ) & G_BGD_MASK )
        bgdQimage.at<Vec3b>(p) = allSpectrum( at, q );
      else if( _gmm.maskAt( at ) & G_FGD_MASK )
        fgdQimage.at<Vec3b>(p) = allSpectrum( at, q );
    }

  imshow( "qimage fgb", fgdQimage );
  imshow( "qimage bgb", bgdQimage );
  imshow( "qimage all", allQimage );
}


// void RMGraphCut::bgRefineMask( float pLimit )
// {
//   CV_Assert( !_image.empty() && !_mask.empty() );
//
//   // Assume the BG mask is more accurate than the FG mask (FG mask might
//   // include some BG).  Update the mask by identifying any PR_FGD points
//   // which are matched by any bgd GMM with at least pLimit.
//   // Switched those points to G_PR_BGD
//
//   Point p;
//   for( p.y = 0; p.y < _image.rows; p.y++ )
//     for( p.x = 0; p.x < _image.cols; p.x++ )
//     {
//       Vec3d color = _csImage.at<Vec3b>(p);
//       if( maskAt(p) != G_PR_FGD ) continue;
//
//       float prob = _bgdGMM.maxQ(color);
//       //LOG(INFO) << p << " : " << prob;
//
//       if( prob > pLimit ) maskSet(p, G_PR_BGD);
//
//     }
//
//   // Relearn the GMMs based
//   initGMMs();
// }
//
// void RMGraphCut::reassignFGtoBG( float pLimit )
// {
//   Point p;
//
//   for( int ci = 0; ci < _fgdGMM.componentsCount(); ++ci ) {
//     int idx;
//     float prob = _bgdGMM.maxQat( _fgdGMM[ci].mean, idx );
//
//     LOG(INFO) << "Fgd " << ci << " : " << _fgdGMM[ci].mean;
//
//     for( int bci = 0; bci < _bgdGMM.componentsCount(); ++bci )
//       LOG(INFO)<< "Bgd " << bci << " : " << _bgdGMM[bci].mean;
//
//
//     LOG(INFO) << "Mean of fgd GMM " << ci << " has max Q = " << prob << " in bgd GMM " << idx;
//
//     if( prob > pLimit ) {
//       LOG(INFO) << "Reassigning FGB " << ci << " to background.";
//
//       for( p.y = 0; p.y < _image.rows; p.y++ )
//         for( p.x = 0; p.x < _image.cols; p.x++ )
//           if( (_fgdGMM.whichComponent( _csImage.at<Vec3b>(p) ) == ci) && (maskAt(p) == G_PR_FGD) )  maskSet(p, G_PR_BGD );
//
//     }
//   }
//
//   // Relearn the GMMs based
//   initGMMs();
// }


bool RMGraphCut::process( int iterCount )
{
  CV_Assert( !_image.empty() && !_labels.empty() );

  if( iterCount <= 0) return false;

  checkLabels( );

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


Mat RMGraphCut::drawLabels( void ) const
{
  Mat image( Mat::zeros( _labels.size(), CV_8UC3 ) );

  const Scalar Red( 0, 0, 255 ), Yellow(0, 255, 255),
                Green(0, 255, 0), Blue(255,0,0),
                Grey( 128,128,128 );

  // My G_* flags are bitwise, so this works.  It doesn't work with the
  // OpenCV original GC_* flags, which aren't bitwise
  image.setTo( Red, _labels & G_FGD );
  image.setTo( Yellow, _labels & G_PR_FGD );
  image.setTo( Green, _labels & G_PR_BGD );
  image.setTo( Blue, _labels & G_BGD );
  image.setTo( Grey, _labels & G_IGNORE );


  return image;
}

//=== Protected functions =================


/*
Check size, type and element values of mask matrix.
*/
void RMGraphCut::checkLabels( void )
{
  if( _labels.empty() )
    CV_Error( CV_StsBadArg, "_labels is empty" );
  if( _labels.type() != CV_8UC1 )
    CV_Error( CV_StsBadArg, "_labels must have CV_8UC1 type" );
  if( _labels.cols != _image.cols || _labels.rows != _image.rows )
    CV_Error( CV_StsBadArg, "_labels must have as many rows and cols as _image" );

  Point p;
  for(  p.y = 0; p.y < _labels.rows; p.y++ )
    for(  p.x = 0; p.x < _labels.cols; p.x++ )
    {
      if( countBits( labelAt(p) ) > 1 )
        CV_Error( CV_StsBadArg, "_labels element value must a single RMGraphCutLabels value" );
    }
}


struct LabelHistogram {
  LabelHistogram( void )
  : fgd(0), bgd(0), ignore(0)  {;}

  unsigned int fgd, bgd, ignore;

  int max( void ) const
  { return std::max( fgd, std::max( bgd, ignore )); }
};

/*
Initialize GMM background and foreground models using kmeans algorithm.
*/
bool RMGraphCut::initGMMs( void )
{
  const int kMeansAttempts = 1;
  const int kMeansIterations = 100;
  const int kMeansType = KMEANS_PP_CENTERS;; //KMEANS_RANDOM_CENTERS; //

  std::vector<Vec3f> samples;
  std::vector<LabelType> labels;

  Point p;
  for( p.y = 0; p.y < _csImage.rows; p.y++ )
    for( p.x = 0; p.x < _csImage.cols; p.x++ ) {
      samples.push_back( (Vec3f)_csImage.at<Vec3b>(p) );
      labels.push_back( labelAt(p) );
    }

  Mat indices;
  //Mat _bgdSamples( (int)bgdSamples.size(), 3, CV_32FC1, &bgdSamples[0][0] );

  kmeans( samples, _gmm.componentsCount(), indices,
          TermCriteria( CV_TERMCRIT_ITER, kMeansIterations, 1e-6), kMeansAttempts, kMeansType );

  vector< LabelHistogram > labelHistograms( _gmm.componentsCount() );

  // Push each cluster into a GMM, then assign each GMM based on current labels
  _gmm.initLearning();
  for( int i = 0; i < (int)samples.size(); i++ )  {
    int index = indices.at<int>(i,0);
    _gmm.addSample( index, samples[i] );

    if( labels[i] & G_BGD_MASK )      ++labelHistograms[ index ].bgd;
    else if( labels[i] & G_FGD_MASK ) ++labelHistograms[ index ].fgd;
    else if( labels[i] & G_IGNORE )   ++labelHistograms[ index ].ignore;
  }

  int fgdSize = labelCount( G_FGD_MASK ),
      ignSize = labelCount( G_IGNORE ),
      bgdSize = _csImage.size().area() - fgdSize - ignSize;

    // Apply labels to each cluster
    for( int i = 0; i < _gmm.componentsCount(); ++i ) {
      float pctFgd = (fgdSize > 0) ? (float)labelHistograms[i].fgd / fgdSize : 0.0,
            pctBgd = (bgdSize > 0) ? (float)labelHistograms[i].bgd / bgdSize : 0.0,
            pctIgn = (ignSize > 0) ? (float)labelHistograms[i].ignore / ignSize : 0.0;

    LOG(INFO) << i << " fgd: " << labelHistograms[i].fgd << " bgd: " << labelHistograms[i].bgd << " ignore: " << labelHistograms[i].ignore;
    LOG(INFO) << i << " fgd: " << pctFgd << " bgd: " << pctBgd << " ignore: " << pctIgn;

    //float theMax = std::max( pctFgd, std::max( pctBgd, pctIgn ));

    if( pctIgn > 0.5 )
      _gmm.setMask( i, G_IGNORE );
    else if( pctFgd > 0.1 )
      _gmm.setMask( i, G_FGD );
    else
      _gmm.setMask( i, G_BGD );

    // if( theMax == pctFgd )
    //   _gmm.setMask( i, G_FGD );
    // else if( theMax == pctBgd )
    //   _gmm.setMask( i, G_BGD );
    // else
    //   _gmm.setMask( i, G_IGNORE );
  }

  _gmm.endLearning();

  return true;
}

/*
Assign GMMs components for each pixel.
*/
void RMGraphCut::assignGMMsComponents( Mat& compIdxs )
{
  Point p;
  for( p.y = 0; p.y < _csImage.rows; p.y++ )
    for( p.x = 0; p.x < _csImage.cols; p.x++ )
    {
      Vec3d color = _csImage.at<Vec3b>(p);

      // Colors are classified within their own type
      MaskedGMM::MaskType label = 0;
      if( labelAt(p) & G_BGD_MASK )      label = G_BGD;
      else if( labelAt(p) & G_FGD_MASK ) label = G_FGD;
      else if( labelAt(p) & G_IGNORE )   label = G_IGNORE;
      else {
        compIdxs.at<int>(p) = -1;
        continue;
      }
      compIdxs.at<int>(p) = _gmm.maxPdfAt( label, color);
    }

}

/*
Updates GMM parameters
*/
void RMGraphCut::learnGMMs( const Mat& compIdxs )
{
  _gmm.initLearning();

  Point p;

  for( p.y = 0; p.y < _csImage.rows; p.y++ )
    for( p.x = 0; p.x < _csImage.cols; p.x++ )
      if( compIdxs.at<int>(p) >= 0 )
        _gmm.addSample( compIdxs.at<int>(p), _csImage.at<Vec3b>(p) );

  _gmm.endLearning( );
}

/*
Construct GCGraph
*/
void RMGraphCut::constructGCGraph( const NeighborWeights &w, GCGraph<double>& graph )
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
    double fromSource = 0.0, toSink = 0.0;
    if( labelAt(p) & (G_PR_BGD | G_PR_FGD ) )
    {
      double ll = _gmm.maxLogLikelihood( G_BGD_MASK, color );
      fromSource = isnan(ll) ? lambda : -ll;

      ll = _gmm.maxLogLikelihood( G_FGD_MASK, color );
      toSink =     isnan(ll) ? lambda : -ll;
    }
    else if( labelAt(p) & G_BGD )
    {
      fromSource = 0;
      toSink = lambda;
    }
    else if( labelAt(p) & G_FGD )
    {
      fromSource = lambda;
      toSink = 0;
    }
    else if( labelAt(p) & G_IGNORE )
    {
      // Something TBD
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
void RMGraphCut::estimateSegmentation( GCGraph<double>& graph )
{
  graph.maxFlow();

  Point p;
  for( p.y = 0; p.y < _labels.rows; p.y++ )
    for( p.x = 0; p.x < _labels.cols; p.x++ )
    {
      // Reassign labels based on segmentation
      if( labelAt(p) &  (G_PR_BGD | G_PR_FGD ) )
      {
        if( graph.inSourceSegment( p.y*_labels.cols + p.x /*vertex index*/ ) )
          setLabel( p, G_PR_FGD );
        else
          setLabel( p, G_PR_BGD );
      }
    }
}


/*
Calculate beta - parameter of GrabCut algorithm.
beta = 1/(2*avg(sqr(||color[i] - color[j]||)))
*/
double RMGraphCut::calcBeta( const Mat& _image )
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
void RMGraphCut::calcNeighborWeights( const Mat& img, NeighborWeights &w )
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
