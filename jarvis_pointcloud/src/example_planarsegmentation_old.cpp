#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//#include <pcl/point_type_conversions.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/region_growing_rgb.h>
//Include these for visualization and filtering ben 11/4
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
// Opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

using namespace std;
using namespace cv;

const string trackbarWindowName = "Trackbars";


//TODO: Filter by distance
//Add visualizer from PCL?
//
ros::Publisher pub;
//create hsv scaler
pcl::visualization::CloudViewer viewer("cloud viewer");

int H_MIN = 0;
int H_MAX = 256;
int S_MIN = 0;
int S_MAX = 256;
int V_MIN = 0;
int V_MAX = 256;

void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed
}

void createTrackbars(){
    //create window for trackbars
    namedWindow(trackbarWindowName,0);
    //create memory to store trackbar name on window
    char TrackbarName[50];
    sprintf( TrackbarName, "H_MIN", H_MIN);
    sprintf( TrackbarName, "H_MAX", H_MAX);
    sprintf( TrackbarName, "S_MIN", S_MIN);
    sprintf( TrackbarName, "S_MAX", S_MAX);
    sprintf( TrackbarName, "V_MIN", V_MIN);
    sprintf( TrackbarName, "V_MAX", V_MAX);
     //create trackbars and insert them into window
    //3 parameters are: the address of the variable that is changing when the trackbar is moved(eg.H_LOW),
      //the max value the trackbar can move (eg. H_HIGH),
        //and the function that is called whenever the trackbar is moved(eg. on_trackbar)
    //                                  ---->    ---->     ---->
    createTrackbar( "H_MIN", trackbarWindowName, &H_MIN, H_MAX, on_trackbar );
    createTrackbar( "H_MAX", trackbarWindowName, &H_MAX, H_MAX, on_trackbar );
    createTrackbar( "S_MIN", trackbarWindowName, &S_MIN, S_MAX, on_trackbar );
    createTrackbar( "S_MAX", trackbarWindowName, &S_MAX, S_MAX, on_trackbar );
    createTrackbar( "V_MIN", trackbarWindowName, &V_MIN, V_MAX, on_trackbar );
    createTrackbar( "V_MAX", trackbarWindowName, &V_MAX, V_MAX, on_trackbar );


}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
    // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGBA>);
    //pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::fromROSMsg (*input, *cloud);
    //make new cloud to operate on
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampled(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZRGBA>);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr rotated(new pcl::PointCloud<pcl::PointXYZRGBA>);

    //downsample
    pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.01f,0.01f,0.01f);
    sor.filter(*downsampled);

// indicies for cropped cloud
    pcl::IndicesPtr indices (new std::vector <int>);
    // Remove anything more than 1.5 meters away
    pcl::PassThrough<pcl::PointXYZRGBA> z_filter;
    z_filter.setInputCloud(downsampled);
    z_filter.setFilterFieldName("z");
    z_filter.setFilterLimits(0.0, 1.0);
    z_filter.filter(*cropped);
    z_filter.filter(*indices);

    pcl::PassThrough<pcl::PointXYZRGBA> x_filter;
    x_filter.setInputCloud(cropped);
    x_filter.setFilterFieldName("x");
    x_filter.setFilterLimits(-0.5,0.5);
    x_filter.filter(*cropped);
    x_filter.filter(*indices);

    pcl::PassThrough<pcl::PointXYZRGBA> y_filter;
    y_filter.setInputCloud(cropped);
    y_filter.setFilterFieldName("y");
    y_filter.setFilterLimits(-0.5,0.5);
    y_filter.filter(*cropped);
    y_filter.filter(*indices);

    //rotate 180
    Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
    //TODO: make the rotation interactive
    //  // Set a rotation around the Z axis.
    float theta = M_PI; // 180 degrees.
    transformation(0, 0) = cos(theta);
    transformation(0, 1) = -sin(theta);
    transformation(1, 0) = sin(theta);
    transformation(1, 1) = cos(theta);
    //
    pcl::transformPointCloud(*cropped, *rotated, transformation);
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_p (new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);
    
    // set up stuff for segmentation
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    //create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
    //optional
    seg.setOptimizeCoefficients (true);
    //Mandatory
   seg.setModelType(pcl::SACMODEL_PLANE);
   seg.setMethodType (pcl::SAC_RANSAC);
   seg.setMaxIterations (1000);
   seg.setDistanceThreshold (0.05);
   //create filtering object
   pcl::ExtractIndices<pcl::PointXYZRGBA> extract;
   int i = 0, nr_points = (int) rotated->points.size ();
   //while 30% of the cloud is still there
  
       // Segment the largest planar component from the remaining cloud
       seg.setInputCloud (rotated);
       seg.segment (*inliers, *coefficients);
       if (inliers->indices.size () == 0)
       {
	   std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
	   
       }
       //extract the inliers
       extract.setInputCloud (rotated);
       extract.setIndices (inliers);
       extract.setNegative (false);
       extract.filter (*cloud_p);
       

       if (!viewer.wasStopped())
       {
	    viewer.showCloud (rotated);
	}
    // Publish the model coefficients
//    pcl_msgs::ModelCoefficients ros_coefficients;
//    pcl_conversions::fromPCL(coefficients, ros_coefficients);
//    pub.publish (ros_coefficients);
}

    int
main (int argc, char** argv)
{
    	cv::namedWindow(OPENCV_WINDOW);
    	  createTrackbars();
    	    cout << "trackbars should exist" << endl;
    // Initialize ROS
    ros::init (argc, argv, "my_pcl_tutorial");

    ros::NodeHandle nh;


    // Create a ROS subscriber for the input point cloud
    ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

    // Create a ROS publisher for the output model coefficients
    pub = nh.advertise<pcl_msgs::ModelCoefficients> ("output", 1);

    // Spin
    ros::spin ();
    //viewer.spinOnce();
}
