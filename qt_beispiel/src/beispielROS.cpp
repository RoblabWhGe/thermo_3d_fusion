/*
 * beispielROS.cpp
 *
 *  Created on: 10.12.2012
 *      Author: Tom-M. Liebelt
 */

#include "beispielROS.h"

beispielROS::beispielROS()
{
	camSub = nodeHandle.subscribe("/image_raw", 1, &beispielROS::camCallback, this);
}

void beispielROS::camCallback(const sensor_msgs::ImageConstPtr& img)
{
	cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(img, "bgr8");
	image = cv_image->image.clone();
}

void beispielROS::sendImage()
{
	emit sigImage(image);
}
