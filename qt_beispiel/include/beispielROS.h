/*
 * beispielROS.h
 *
 *  Created on: 10.12.2012
 *      Author: Tom-M. Liebelt
 */

#ifndef BEISPIELROS_H_
#define BEISPIELROS_H_

#include <QObject>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>

class beispielROS : public QObject
{
	Q_OBJECT

public:
	beispielROS();

signals:
	void sigImage(cv::Mat image);

public slots:
	void sendImage();

private:
	ros::NodeHandle nodeHandle;
	ros::Subscriber camSub;
	cv::Mat image;
	void camCallback(const sensor_msgs::ImageConstPtr& img);
};

#endif /* BEISPIELROS_H_ */
