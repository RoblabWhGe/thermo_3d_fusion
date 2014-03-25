/*
 * main.cpp
 *
 *  Created on: 20.02.2013
 *      Author: Tom-M. Liebelt
 */


#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread.hpp>
#include "ViewerROS.h"
#include "ViewerGUI.h"
#include <QApplication>

int main (int argc, char** argv)
{
	ros::init(argc, argv, "thermo_3d_viewer");
	QApplication app(argc, argv);
	ViewerROS viewROS;
	ViewerGUI viewGUI;
	QObject::connect(&viewROS, SIGNAL(got3DThermoData(pcl::PointCloud<pcl::PointXYZRGB> &,
													  std::vector<float> &)),
				     &viewGUI, SLOT(updatePointCloud(pcl::PointCloud<pcl::PointXYZRGB> &,
				    		 	 	 	 	 	 	  std::vector<float> &)));

	while (ros::ok())
	{
		ros::spinOnce();
		QCoreApplication::processEvents();
		viewGUI.pclSpinOnce(100);
		boost::this_thread::sleep (boost::posix_time::microseconds (100000));
	}
    return 0;
}


