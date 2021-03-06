#ifndef __GRAPHCUT_H__
#define __GRAPHCUT_H__


#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "gcgraph.hpp"

#include "gmm.h"

using cv::Rect;
using cv::Point;
using cv::Mat;

// void graphCut( InputArray img, InputOutputArray mask, Rect rect,
//   InputOutputArray bgdModel, InputOutputArray fgdModel,
//   int iterCount, int mode = cv::GC_EVAL );

namespace GC {
  // Shared with RMGraphCut
  enum GraphCutLabels {
    G_MASK   = 0,
    G_BGD    = 1,
    G_PR_BGD = 2,
    G_BGD_MASK = (G_PR_BGD | G_BGD),
    G_PR_FGD = 4,
    G_FGD    = 8,
    G_FGD_MASK = (G_PR_FGD | G_FGD),
    G_IGNORE   = 128,
    G_ALL    = 0xFF
  };

  typedef uchar LabelType;
}


class GraphCut {
public:
  typedef GC::LabelType LabelType;

  // gamma = 50 from original Grabcut paper
  GraphCut( double colorWeight = 50 );

  void setMask( const Mat &mask );
  void setMaskRect( const Rect &rect );
  void setImage( const Mat &img );

  void showMaxQImages( );
  void bgRefineMask( float pLimit = 0.5 );
  void reassignFGtoBG( float pLimit = 1e-4 );

  bool process( int iterCount = 1 );

  const Mat &mask( void ) const { return _mask; }
  Mat fgdMask( void ) const {  return  ( mask() & GC::G_FGD_MASK ); }

  Mat drawMask( void ) const;

  LabelType maskAt( const Point &p ) const           { return _mask.at<LabelType>(p); }
  void      maskSet( const Point &p, LabelType l )   { _mask.at<LabelType>(p) = l; }

  struct NeighborWeights {
    Mat left, upleft, up, upright;
  };

protected:

  void checkMask( void );

  bool initGMMs( void );
  void assignGMMsComponents( Mat& compIdxs );
  void learnGMMs( const Mat& compIdxs );
  void constructGCGraph( const NeighborWeights &w,
                         GCGraph<double>& graph );

    void estimateSegmentation( GCGraph<double>& graph );

    double calcBeta( const Mat& _image );
    void calcNeighborWeights( const Mat& img, NeighborWeights &w );

    // This is "gamma" in the original Grabcut algorithm, the weight on the color
// difference term.
  double _colorWeight;

    Mat _image, _csImage, _mask;

    GMM _ignoreGMM, _bgdGMM, _fgdGMM;
  };

  #endif
