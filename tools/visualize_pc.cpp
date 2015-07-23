

#include <glog/logging.h>


#include "visualize_app_common.h"

using namespace std;
using namespace cv;
using namespace Distortion;


class PCVisualizerOpts : public VisualizerOpts {
public:
  PCVisualizerOpts( void )
  {;}


  virtual void doParseCmdLine( TCLAP::CmdLine &cmd )
  {
    VisualizerOpts::doParseCmdLine( cmd );
  }

  virtual bool  validate( void )
  {
    return true;
  }

  bool doAnnotate( void )
  {
    return annotateMode != NONE;
  }

};

static void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event, void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

LOG(INFO) << "Keypress: " << event.getKeySym();

if (event.getKeySym () == "q" && event.keyDown ()) {
    viewer->close();
  }
}

static void mouseEventOccurred (const pcl::visualization::MouseEvent &event,  void* viewer_void)
{
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);

  // if (event.getButton () == pcl::visualization::MouseEvent::LeftButton &&
  // event.getType () == pcl::visualization::MouseEvent::MouseButtonRelease)
  // {
  //   std::cout << "Left mouse button released at position (" << event.getX () << ", " << event.getY () << ")" << std::endl;
  //
  //   char str[512];
  //   sprintf (str, "text#%03d", text_id ++);
  //   viewer->addText ("clicked here", event.getX (), event.getY (), str);
  // }
}


class PCVisualizer : public VisualizerCommon  {
public:

  PCVisualizer( PCVisualizerOpts &opts_ )
  : VisualizerCommon( opts_ ), opts(opts_)
  {;}

  virtual ~PCVisualizer()
  {
  }

  virtual int run( void )
  {

    VisualizerCommon::loadModels();

    doVisualize();


    //  // ----------------------------------------------------------------
    //  // -----Calculate surface normals with a search radius of 0.05-----
    //  // ----------------------------------------------------------------
    //  pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    //  ne.setInputCloud (point_cloud_ptr);
    //  pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB> ());
    //  ne.setSearchMethod (tree);
    //  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals1 (new pcl::PointCloud<pcl::Normal>);
    //  ne.setRadiusSearch (0.05);
    //  ne.compute (*cloud_normals1);
    //
    //  // ---------------------------------------------------------------
    //  // -----Calculate surface normals with a search radius of 0.1-----
    //  // ---------------------------------------------------------------
    //  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
    //  ne.setRadiusSearch (0.1);
    //  ne.compute (*cloud_normals2);




    return 0;
  }

  int doVisualize( void )
  {
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));

    viewer->setBackgroundColor (0, 0, 0);

    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud_ptr);
    viewer->addPointCloud<pcl::PointXYZRGB> ( cloud_ptr, rgb, "sample cloud");

    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (0.25, 0, 0, 0 );
    viewer->initCameraParameters ();

    // If imageAxes, attempt to align camera so it's still "upright"
    // that is, initial view, +X is to right, +Y is down, +Z is away
    if( opts.imageAxes ) viewer->setCameraPosition( 0, 0, 0, 0, -1, 0 );

    viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
    viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

    //--------------------
    // -----Main loop-----
    //--------------------
    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }

    return 0;
  }


protected:

  PCVisualizerOpts &opts;

};


// --------------
// -----Main-----
// --------------
int main (int argc, char** argv)
{
  google::InitGoogleLogging( argv[0] );
  FLAGS_logtostderr = 1;

  PCVisualizerOpts opts;
  if( !opts.parseCmdLine( argc, argv ) ) exit(-1);

  PCVisualizer viz( opts );
  return viz.run();
}



#ifdef FALSE

boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> customColourVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> single_color(cloud, 0, 255, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, single_color, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> normalsVis (
  pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals)
  {
    // --------------------------------------------------------
    // -----Open 3D viewer and add point cloud and normals-----
    // --------------------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    return (viewer);
  }


  boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
  {
    // --------------------------------------------
    // -----Open 3D viewer and add point cloud-----
    // --------------------------------------------
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();

    //------------------------------------
    //-----Add shapes at cloud points-----
    //------------------------------------
    viewer->addLine<pcl::PointXYZRGB> (cloud->points[0],
      cloud->points[cloud->size() - 1], "line");
      viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

      //---------------------------------------
      //-----Add shapes at other locations-----
      //---------------------------------------
      pcl::ModelCoefficients coeffs;
      coeffs.values.push_back (0.0);
      coeffs.values.push_back (0.0);
      coeffs.values.push_back (1.0);
      coeffs.values.push_back (0.0);
      viewer->addPlane (coeffs, "plane");
      coeffs.values.clear ();
      coeffs.values.push_back (0.3);
      coeffs.values.push_back (0.3);
      coeffs.values.push_back (0.0);
      coeffs.values.push_back (0.0);
      coeffs.values.push_back (1.0);
      coeffs.values.push_back (0.0);
      coeffs.values.push_back (5.0);
      viewer->addCone (coeffs, "cone");

      return (viewer);
    }


    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewportsVis (
      pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud, pcl::PointCloud<pcl::Normal>::ConstPtr normals1, pcl::PointCloud<pcl::Normal>::ConstPtr normals2)
      {
        // --------------------------------------------------------
        // -----Open 3D viewer and add point cloud and normals-----
        // --------------------------------------------------------
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->initCameraParameters ();

        int v1(0);
        viewer->createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer->setBackgroundColor (0, 0, 0, v1);
        viewer->addText("Radius: 0.01", 10, 10, "v1 text", v1);
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud1", v1);

        int v2(0);
        viewer->createViewPort(0.5, 0.0, 1.0, 1.0, v2);
        viewer->setBackgroundColor (0.3, 0.3, 0.3, v2);
        viewer->addText("Radius: 0.1", 10, 10, "v2 text", v2);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> single_color(cloud, 0, 255, 0);
        viewer->addPointCloud<pcl::PointXYZRGB> (cloud, single_color, "sample cloud2", v2);

        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud1");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud2");
        viewer->addCoordinateSystem (1.0);

        viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals1, 10, 0.05, "normals1", v1);
        viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals2, 10, 0.05, "normals2", v2);

        return (viewer);
      }




      boost::shared_ptr<pcl::visualization::PCLVisualizer> interactionCustomizationVis ()
      {
        boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer->setBackgroundColor (0, 0, 0);
        viewer->addCoordinateSystem (1.0);

        viewer->registerKeyboardCallback (keyboardEventOccurred, (void*)&viewer);
        viewer->registerMouseCallback (mouseEventOccurred, (void*)&viewer);

        return (viewer);
      }

      #endif
