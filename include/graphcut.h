#ifndef __GRAPHCUT_H__
#define __GRAPHCUT_H__


#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "gcgraph.hpp"

#include "gmm.h"

using cv::Rect;
using cv::Mat;

// void graphCut( InputArray img, InputOutputArray mask, Rect rect,
//   InputOutputArray bgdModel, InputOutputArray fgdModel,
//   int iterCount, int mode = cv::GC_EVAL );


class GraphCut {
public:

  GraphCut( void );

  void setMask( const Mat &mask );
  void setMaskRect( const Rect &rect );
  void setImage( const Mat &img );

void showMaxQImages( );
void bgRefineMask( float pLimit = 0.5 );

  void process( int iterCount = 1 );

  const Mat &mask( void ) const { return _mask; }

Mat drawMask( void ) const;

protected:

  void checkMask( void );

  void initGMMs( void );
  void assignGMMsComponents( Mat& compIdxs );
  void learnGMMs( const Mat& compIdxs );
  void constructGCGraph( double lambda,
    const Mat& leftW, const Mat& upleftW, const Mat& upW, const Mat& uprightW,
    GCGraph<double>& graph );

    void estimateSegmentation( GCGraph<double>& graph );

    double calcBeta( const Mat& _image );
    void calcNWeights( const Mat& img, Mat& leftW, Mat& upleftW, Mat& upW, Mat& uprightW, double beta, double gamma );


    Mat _image, _csImage, _mask, _fgdModel, _bgdModel;
    GMM<5> _bgdGMM;
    GMM<5> _fgdGMM;
  };

  #endif
