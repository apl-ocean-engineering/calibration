#ifndef __RM_GRAPHCUT_H__
#define __RM_GRAPHCUT_H__


#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "gcgraph.hpp"

#include "gmm.h"

using cv::Rect;
using cv::Point;
using cv::Mat;



class RMGraphCut {
public:

  enum GraphCutLabels {
    G_MASK   = 0,
    G_BGD    = 1,
    G_PR_BGD = 2,
    G_BGD_MASK = 0x03,
    G_PR_FGD = 4,
    G_FGD    = 8,
    G_FGD_MASK = 0x0C,
    G_IGNORE   = 128
  };
  typedef uchar LabelType;

  // gamma = 50 from original Grabcut paper
  RMGraphCut( double colorWeight = 50 );

  void setLabels( const Mat &labels );
  void setImage( const Mat &img );

  void showMaxQImages( );
  // void bgRefineMask( float pLimit = 0.5 );
  // void reassignFGtoBG( float pLimit = 1e-4 );

  bool process( int iterCount = 1 );

  const Mat &labels( void ) const     { return _labels; }
  Mat labels( LabelType mask ) const  { return ( labels() & mask ); }
  Mat fgdLabels( void ) const         {  return labels( G_FGD_MASK ); }
  unsigned int labelCount( LabelType mask );

  Mat drawLabels( void ) const;

  LabelType labelAt( const Point &p ) const           { return _labels.at<LabelType>(p); }
  void      setLabel( const Point &p, LabelType l )   { _labels.at<LabelType>(p) = l; }

  struct NeighborWeights {
    Mat left, upleft, up, upright;
  };

protected:

  void checkLabels( void );

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

    Mat _image, _csImage, _labels;
    MaskedGMM _gmm;
  };

  #endif
