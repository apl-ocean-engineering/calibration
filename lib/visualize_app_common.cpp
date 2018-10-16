
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include "visualize_app_common.h"

namespace camera_calibration {


  void VisualizerCommon::filterSmallClusters( void )
  {
    pcl::RadiusOutlierRemoval<PCPointType> rorfilter( true ); // Initializing with true will allow us to extract the removed indices

    LOG(INFO) << "Before small cluster filtering, inlier point cloud has " << inliers->size() << " elements, outlier has " << outliers->size();
    rorfilter.setInputCloud( inliers );
    rorfilter.setRadiusSearch (opts.smallClusterRadius);
    rorfilter.setMinNeighborsInRadius(opts.smallClusterNeighbors);
    //rorfilter.setNegative (false);

    // Is this the hardest way to do this?
    pcl::IndicesPtr indices( new std::vector<int> );
    rorfilter.filter(*indices);

    // Extract the inliers/outliers
    pcl::ExtractIndices< PCPointType > extract;
    extract.setInputCloud( inliers );
    extract.setIndices( indices );

    pcl::PointCloud< PCPointType > inCluster, outCluster;
    extract.setNegative( false );
    extract.filter( inCluster );
    extract.setNegative( true );
    extract.filter( outCluster );

    inliers->swap( inCluster );
    (*outliers) += outCluster;

    LOG(INFO) << "After small cluster filter, it has " << inliers->size() << " elements, outlier has " << outliers->size();
    //indices_rem = rorfilter.getRemovedIndices ();
  }

}
