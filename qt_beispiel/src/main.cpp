/*
 * main.cpp
 *
 *  Created on: 10.12.2012
 *      Author: Tom-M. Liebelt
 */

#include <ros/ros.h>
#include "beispielGUI.h"
#include "beispielROS.h"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "qt_beispiel");

	beispielROS ros;
	QApplication app(argc, argv);
	beispielGUI gui;

	QObject::connect(&ros, SIGNAL(sigImage(cv::Mat)), &gui, SLOT(showImage(cv::Mat)));
	QObject::connect(&gui, SIGNAL(nextImage()), &ros, SLOT(sendImage()));

	ros::Rate loop_rate(50);
	while (ros::ok())
	{
		QCoreApplication::processEvents();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
