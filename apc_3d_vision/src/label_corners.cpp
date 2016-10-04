#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <boost/shared_ptr.hpp>
#include <cstdlib>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

//Can't find these
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/point_picking_event.h>

//Global VARIABLES

int ii = 0; //Counter

float x_vals[4];
float y_vals[4];
float z_vals[4];

//Create PointXYZ pointer
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ> > cloud;
//Create PointXYZL pointer
boost::shared_ptr<pcl::PointCloud<pcl::PointXYZL> > labelled_points;

//Function Declarations:
void pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void);


//Main
int main (){

    //Set up PCL reader
    pcl::PCDReader reader;
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);

    cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);   //Not sure what we're doing here but it seems necessary.
    labelled_points.reset(new pcl::PointCloud<pcl::PointXYZL>);     //Not sure what we're doing here but it seems necessary.

    //Load in PCL file
    reader.read ("test.pcd", *cloud);

    //Copy contents of original XYZ Point Cloud to new XYZL Point Cloud
    pcl::copyPointCloud(*cloud, *labelled_points);


    //Guessing
    pcl::visualization::PCLVisualizer visualizer("PCL visualizer");
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud, 255, 255, 255);
    visualizer.addPointCloud (cloud, cloud_color_handler, "original_cloud");
    visualizer.registerPointPickingCallback(pp_callback, (void*)&visualizer);
    visualizer.spin ();

    //To save data to pcd file:
    pcl::io::savePCDFileASCII ("labelled_point_cloud.pcd", *labelled_points);




    /////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    /*
    // Visualization
    pcl::visualization::PCLVisualizer viewer ("Guessing");

     // Define R,G,B colors for the point cloud
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color_handler (cloud, 255, 255, 255);
    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (cloud, cloud_color_handler, "original_cloud");

    viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");
    //viewer.setPosition(800, 400); // Setting visualiser window position

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
      viewer.spinOnce ();
    }
    */
    /////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////
    ////////////////////////////////////////////////////////////////////////////
    //std::cout << EXIT_SUCCESS << std::endl;
    return EXIT_SUCCESS;
}


//Script should do:
//Open pcd file
//boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> points;
/*
points.reset(new pcl::PointCloud<pcl::PointXYZ>);

//Convert the loaded pcd file to a xyzl type.

boost::shared_ptr<pcl::PointCloud<pcl::PointXYZL>> labelled_points;

labelled_points.reset(new pcl::PointCloud<pcl::PointXYZL>);

pcl::copyPointCloud(*points, *labelled_points);

//Open a viewer window of the labelled point cloud

//http://stackoverflow.com/questions/26699427/checking-point-coordinates-in-pclvisualizer

//http://docs.pointclouds.org/trunk/classpcl_1_1visualization_1_1_point_picking_event.html

void pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void)

{

   std::cout << "Picking event active" << std::endl;

   labelled_points[event.getPointIndex()]->label = 1;

   /*

   if(event.getPointIndex()!=-1)

   {

       float x,y,z;

       event.getPoint(x,y,z);

       std::cout << x<< ";" << y<<";" << z << std::endl;

   }

   */

//}

//save labelled_points to a pcd file

//look at the file to see if it worked (or open the pcd in pcl_viewer)

//For example on interfacing to xyzl point clouds, have a look in BaxterPicks/Apc3dVision/src/split_labelled_point_cloud_ros_node.cpp




//Functions
void pp_callback(const pcl::visualization::PointPickingEvent& event, void* viewer_void){
   std::cout << "Picking event active" << std::endl;
   //labelled_points[event.getPointIndex()]->label = 1;
   if(event.getPointIndex()!=-1)
   {
       //float x,y,z;
       //event.getPoint(x,y,z);
       //std::cout << x<< ";" << y<<";" << z << std::endl;
       if (ii < 4){


           ii++;
           labelled_points[event.getPointIndex()].label = ii;
           /*
           //Load in PCL file
           //reader.read ("test.pcd", *cloud);
           x_vals[ii];
           y_vals[ii];
           z_vals[ii];
           */

        }
       else{
           std::cout << "Only four points need to be selected." << std::endl;
           //std::cout << "Please re-select the points" << std::endl;
           //ii = 0;
       }
       ////////////////////////////
       /*
       //Store Points in Matrix:
       //[x0,x1,x2,x3
       // y0,y1,y2,y3
       // z0,z1,z2,z3]
       points[0][ii] = x;
       points[1][ii] = y;
       points[2][ii] = z;
       ii+;
       */
       ////////////////////////////
   }
}
