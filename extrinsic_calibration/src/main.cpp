#include "ExtrinsicCalibrationGUI.h"
#include "ExtrinsicCalibrationROS.h"
#include <QApplication>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "extrinsic_calibration");

	ExtrinsicROS ros;
	QApplication app(argc, argv);
	ExtrinsicGUI gui;

	QObject::connect(&ros, SIGNAL(gotImageLS(cv::Mat)), &gui, SLOT(setImageLS(cv::Mat)));
	QObject::connect(&gui, SIGNAL(gotAllCornersLS(std::vector<cv::Point2f>)), &ros, SLOT(addPointsLS(std::vector<cv::Point2f>)));

	QObject::connect(&ros, SIGNAL(gotImageC(cv::Mat)), &gui, SLOT(setImageC(cv::Mat)));
	QObject::connect(&gui, SIGNAL(gotAllCornersC(std::vector<cv::Point2f>)), &ros, SLOT(addPointsC(std::vector<cv::Point2f>)));
	QObject::connect(&gui, SIGNAL(nextImageC()), &ros, SLOT(nextImageC()));
	QObject::connect(&gui, SIGNAL(prevImageC()), &ros, SLOT(prevImageC()));

	ros::Rate loop_rate(50);
	while (ros::ok() && !ros.calibrate())
	{
		QCoreApplication::processEvents();
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}
