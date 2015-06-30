#ifndef __SONAR_DETECTIONS_H__
#define __SONAR_DETECTIONS_H__

#include <vector>
#include <glog/logging.h>

#include "detection_base.h"
#include "image_detections.h"
#include "sonar_image_warper.h"

class SonarDetection : public DetectionBase {
public:
  SonarDetection( const std::string &timestamp )
  : DetectionBase( timestamp )
  {;}

  virtual ImageDetection *projectToImage( SonarImageWarper *warper ) = 0;

};

class SonarSphereDetection : public SonarDetection {
public:
  SonarSphereDetection( const std::string &timestamp, float x, float y, float z, float radius )
  : SonarDetection( timestamp ),
  _x(x), _y(y), _z(z), _radius(radius)
  {;}

  virtual ImageDetection *projectToImage( SonarImageWarper *warper );

protected:

  float _x, _y, _z, _radius;

};

class SonarDetections {
public:

  SonarDetections()
  {;}

  SonarDetections( const std::string &filename )
  {
    load( filename );
  }

  SonarDetection *find( const std::string &timestamp );

  void load( const std::string &filename, bool imageAxes = false );

protected:

  typedef std::vector< SonarDetection * > DetectionVec;
  DetectionVec _dets;

};

#endif
