/*

* utility_functions.cpp
 *
 *  Created on: Nov 16, 2014
 *      Author: ben
 */
void downSample(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr downsampled)
{
	pcl::VoxelGrid<pcl::PointXYZRGBA> sor;
	    sor.setInputCloud(&cloud);
	    sor.setLeafSize(0.01f,0.01f,0.01f);
	    sor.filter(downsampled);
}

void test()
{
	cout << "TESTEST" << endl;
}

