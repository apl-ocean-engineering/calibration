
#include <pcl/filters/radius_outlier_removal.h>

#include "visualize_app_common.h"



void VisualizerCommon::filterSmallClusters( void )
{
  pcl::RadiusOutlierRemoval<PCPointType> rorfilter( true ); // Initializing with true will allow us to extract the removed indices

  rorfilter.setInputCloud (cloud_ptr);
  rorfilter.setRadiusSearch (opts.smallClusterRadius);
  rorfilter.setMinNeighborsInRadius(opts.smallClusterNeighbors);
  rorfilter.setNegative (false);

  rorfilter.filter (*cloud_ptr);

  //indices_rem = rorfilter.getRemovedIndices ();
}
