/*
 * viewer.cpp
 *
 *  Created on: Nov 16, 2014
 *      Author: ben
 */
 #include <pcl/visualization/cloud_viewer.h>
#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/visualization/pcl_visualizer.h"
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
// Opencv
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#define MIN(x, y) ( (x) <= (y) ) ? (x) : (y)
#define MAX(x, y) ( (x) >= (y) ) ? (x) : (y)

using namespace std;
using namespace cv;


const string trackbarWindowName = "Trackbars";
int H_MIN = 0;
int H_MAX = 360;
int S_MIN = 0;
int S_MAX = 100;
int V_MIN = 0;
int V_MAX = 100;

void on_trackbar( int, void* )
{//This function gets called whenever a
	// trackbar position is changed
}

void createTrackbars(){
	cout << "Trackbars" << endl;
    //create window for trackbars
    cv::namedWindow("HSV_vals");
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
    cvCreateTrackbar( "H_MIN", "HSV_vals", &H_MIN, H_MAX, NULL );
    cvCreateTrackbar( "H_MAX", "HSV_vals", &H_MAX, H_MAX, NULL);
    cvCreateTrackbar( "S_MIN", "HSV_vals", &S_MIN, S_MAX, NULL );
    cvCreateTrackbar( "S_MAX", "HSV_vals", &S_MAX, S_MAX, NULL );
    cvCreateTrackbar( "V_MIN", "HSV_vals", &V_MIN, V_MAX, NULL );
    cvCreateTrackbar( "V_MAX", "HSV_vals", &V_MAX, V_MAX, NULL );


}

void downSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled)
{
	pcl::VoxelGrid<pcl::PointXYZRGB> sor;
	    sor.setInputCloud(cloud);
	    sor.setLeafSize(0.01f,0.01f,0.01f);
	    sor.filter(*downsampled);
}

void cropCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped, pcl::IndicesPtr indices)
{
	// Remove anything more than 1.5 meters away
	    pcl::PassThrough<pcl::PointXYZRGB> z_filter;
	    z_filter.setInputCloud(cloud);
	    z_filter.setFilterFieldName("z");
	    z_filter.setFilterLimits(0.0, 1.0);
	    z_filter.filter(*cropped);
	    z_filter.filter(*indices);

	    pcl::PassThrough<pcl::PointXYZRGB> x_filter;
	    x_filter.setInputCloud(cropped);
	    x_filter.setFilterFieldName("x");
	    x_filter.setFilterLimits(-0.5,0.5);
	    x_filter.filter(*cropped);
	    x_filter.filter(*indices);

	    pcl::PassThrough<pcl::PointXYZRGB> y_filter;
	    y_filter.setInputCloud(cropped);
	    y_filter.setFilterFieldName("y");
	    y_filter.setFilterLimits(-0.5,0.5);
	    y_filter.filter(*cropped);
	    y_filter.filter(*indices);
}
//rotate around the z axis
void rotateCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated)
{
	 Eigen::Matrix4f transformation = Eigen::Matrix4f::Identity();
	    //TODO: make the rotation interactive
	    //  // Set a rotation around the Z axis.
	    float theta = M_PI; // 180 degrees.
	    transformation(0, 0) = cos(theta);
	    transformation(0, 1) = -sin(theta);
	    transformation(1, 0) = sin(theta);
	    transformation(1, 1) = cos(theta);
	    //
	    pcl::transformPointCloud(*cloud, *rotated, transformation);
}

void toHSV(int ri, int gi, int bi, float *h, float *s, float *v)
{
	// 0 < h < 360, 0<s<1 , 0<v<1
	float r, g, b;
	r = ri/255.0;
	g = gi/255.0;
	b = bi/255.0;

	float mn, mx, delta;
	mn = MIN(r, MIN(g, b));
	mx = MAX(r, MAX(g, b));


	*v = mx;
	delta = mx - mn;

	if (mx != 0)
		*s = delta/mx;
	else
	{
		*s = 0;
		*h = -1;
		return;
	}

	if (r == mx)
		*h = (g-b)/delta;
	else if (g == mx)
		*h = 2 + (b-r)/delta;
	else
		*h = 4 + (r-g)/delta;

	*h *= 60;
	if (*h < 0)
		*h += 360;
	//cout << "h = " << *h << " s= " << *s << " v= " << *v <<endl;
}

void colorSegment(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input,
		pcl::PointCloud<pcl::PointXYZRGB>::Ptr hand_cloud)
{
	float range[][2] = {{H_MIN/1.0,H_MAX/1.0},
		{S_MIN/100.0, S_MAX/100.0},
		{V_MIN/100.0, V_MAX/100.0}};


	for (int i = 0; i < input->points.size(); i++)
	{
		float h, s, v;
		toHSV(input->points[i].r, input->points[i].g, input->points[i].b, &h, &s, &v);
		//cout << "h = " << h << " s= " << s << " v= " << v <<endl;
		//cout << "h_range = " << range[0][0] << "to " << range[0][1] << endl;
		//cout << "s_range = " << range[1][0] << "to " << range[1][1] << endl;
		//cout << "v_range = " << range[2][0] << "to " << range[2][1] << endl;
		if ( (h > range[0][0]) && (h < range[0][1]) &&
				(s > range[1][0]) && (s < range[1][1]) &&
				(v > range[2][0]) && (v < range[2][1]))
		{
			hand_cloud->push_back(input->points[i]);
		}
	}
}

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

 class SimpleOpenNIViewer
 {
   public:

	 ros::Subscriber sub;

     pcl::visualization::CloudViewer viewer;
     ros::NodeHandle& nh_;
     bool cloud_received;

	 pcl::PointCloud<pcl::PointXYZ> cloud;

	 SimpleOpenNIViewer (ros::NodeHandle& nh) : viewer ("PCL OpenNI Viewer"), nh_(nh),cloud_received(false) {
		  sub = nh.subscribe("/camera/depth_registered/points", 1, &SimpleOpenNIViewer::cloud_cb_, this);
		  createTrackbars();
	 }

     void cloud_cb_ (const sensor_msgs::PointCloud2ConstPtr& input)
     {

    	 if (!viewer.wasStopped()){

    		// if(!cloud_received){


    			 // Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
    			     pcl::PointCloud<pcl::PointXYZRGB>::Ptr in_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    			     //pcl::PointCloud<pcl::PointXYZ> cloud;
    			     pcl::fromROSMsg (*input, *in_cloud);
    			     //downsample
    			     pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    			     downSample(in_cloud, downsampled_cloud);
    			     //crop
    			     pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    			     pcl::IndicesPtr indices (new std::vector <int>);
    			     cropCloud(downsampled_cloud, cropped_cloud, indices);
    			     //rotate

    			     pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    			     rotateCloud(cropped_cloud, rotated_cloud);
    			     pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    			     colorSegment(rotated_cloud, filtered_cloud);



    			     /*
    		 pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    		  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    		  // Create the segmentation object
    		  pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    		  // Optional
    		  seg.setOptimizeCoefficients (true);
    		  // Mandatory
    		  seg.setModelType (pcl::SACMODEL_PLANE);
    		  seg.setMethodType (pcl::SAC_RANSAC);
    		  seg.setDistanceThreshold (0.01);

    		  seg.setInputCloud (msg);
    		  seg.segment (*inliers, *coefficients);

    		  if (inliers->indices.size () == 0)
    		  {
    		    PCL_ERROR ("Could not estimate a planar model for the given dataset.");
    		    return;
    		    //return (-1);
    		  }

    		  std::cerr << "Model coefficients: " << coefficients->values[0] << " "
    		                                      << coefficients->values[1] << " "
    		                                      << coefficients->values[2] << " "
    		                                      << coefficients->values[3] << std::endl;

    		  std::cerr << "Model inliers: " << inliers->indices.size () << std::endl;

    		  cloud.points.resize(inliers->indices.size ());
    		  cloud.width=1;
    		  cloud.height= inliers->indices.size ();
    		  for (size_t i = 0; i < inliers->indices.size (); ++i){
    			  cloud.points[i].x = (*msg).points[inliers->indices[i]].x;
    			  cloud.points[i].y = (*msg).points[inliers->indices[i]].y;
    			  cloud.points[i].z = (*msg).points[inliers->indices[i]].z;

    			  //std::cerr << inliers->indices[i] << "    " << (*msg).points[inliers->indices[i]].x << " "
    		      //                                         << (*msg).points[inliers->indices[i]].y << " "
    		      //                                         << (*msg).points[inliers->indices[i]].z << std::endl;
    		  }
              //cloud_received = true;
    		}
    		*/
    		  //return (0);
    	     cv::waitKey(3);
             viewer.showCloud(filtered_cloud);
             //viewer.showCloud(msg);
    		 }

     }
 };

 int main(int argc, char** argv)

 {

	  ros::init(argc, argv, "sub_pcl");

	  ros::NodeHandle nh;
	  SimpleOpenNIViewer v(nh);
	  ros::spin();


   return 0;
 }




