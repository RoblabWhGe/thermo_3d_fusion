/*
 * viewerROS.cpp
 *
 *  Created on: 20.02.2013
 *      Author: Tom-M. Liebelt
 */

#include <ros/ros.h>
#include <QObject>
#include <sensor_msgs/PointCloud2.h>
#include <thermo_cloud/ThermoData.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/ros/conversions.h>

class ViewerROS : public QObject
{
	Q_OBJECT

public:
	ViewerROS() :
			cloudSub(nH, "/3d_thermo/rgb_cloud", 1),
			tempSub(nH, "/3d_thermo/thermo_data", 1),
			sync(cloudSub, tempSub, 10)
	{
		sync.registerCallback(boost::bind(&ViewerROS::callback, this, _1, _2));
	}

	~ViewerROS(){}

Q_SIGNALS:

	void got3DThermoData(pcl::PointCloud<pcl::PointXYZRGB> &newCloud, std::vector<float> &newTemps);

private:
	ros::NodeHandle nH;

	message_filters::Subscriber<sensor_msgs::PointCloud2> cloudSub;
	message_filters::Subscriber<thermo_cloud::ThermoData> tempSub;
	message_filters::TimeSynchronizer<sensor_msgs::PointCloud2, thermo_cloud::ThermoData> sync;

	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	thermo_cloud::ThermoData temps;

	void callback(const sensor_msgs::PointCloud2ConstPtr& cloudP,
				  const thermo_cloud::ThermoDataConstPtr& tempsP)
	{
		pcl::fromROSMsg(*cloudP, cloud);
		temps = *tempsP;

		if (cloud.size() == temps.width)
		{
			Q_EMIT got3DThermoData(cloud, temps.data);
		}
	}
};

