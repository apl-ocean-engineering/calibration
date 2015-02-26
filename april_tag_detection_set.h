
#include <vector>

#include <AprilTags/TagDetection.h>

#ifndef __APRIL_TAG_DETECTION_SET_H__
#define __APTIL_TAG_DETECTION_SET_H__

using AprilTags::TagDetection;
using std::vector;


class AprilTagDetectionSet {
  public:
    struct DetectionNode 
    {
      public:
        DetectionNode()
          : up( -1 ), down( -1 ), left( -1 ), right( -1 )
        {;}

        int up, down, left, right;
    };


    AprilTagDetectionSet( const vector<TagDetection> detections );

    cv::Mat gridOfIds( void );
    cv::Mat gridOfIndices( void ) const { return _grid; }

    const TagDetection &operator[]( int idx ) const { return _detections[idx]; }

    const TagDetection &at( int x, int y ) const 
    { return _detections[ indexAt( x, y )]; }

    int indexAt( int x, int y ) const 
    { return _grid.at<int16_t>(y,x); }

    bool validAt( int x, int y ) const
    { return indexAt(x,y) >= 0; }


  private:

    void arrangeIntoGrid( void );

    void assignToGraph( const vector<DetectionNode> &nodes, const int idx, 
                        cv::Mat &graph, const int x, const int y, int *limits );

    const vector<TagDetection> _detections;
    cv::Mat _grid;

};

#endif
