
#include <vector>

#include <AprilTags/TagDetection.h>

#ifndef __APRIL_TAG_DETECTION_SET_H__
#define __APTIL_TAG_DETECTION_SET_H__

using AprilTags::TagDetection;
using std::vector;


class AprilTagDetectionSet {
  public:

    AprilTagDetectionSet( const vector<TagDetection> detections );

    cv::Mat filterByHomography( void );


  private:


    vector<TagDetection> _detections;


};

#endif
