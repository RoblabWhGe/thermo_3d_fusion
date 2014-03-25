/*
 * ExtrinsicCalibrationROS.cpp
 *
 *  Created on: 11.01.2013
 *      Author: Tom-M. Liebelt
 */

#include "ExtrinsicCalibrationROS.h"

using namespace cv;
using namespace tf;
using namespace std;

const double ExtrinsicROS::ANGLE_BTP_WIDTH = 0.25;
const double ExtrinsicROS::ANGLE_BTP_HEIGHT = 0.25;
const int ExtrinsicROS::LASER_WIDTH = 360 / ANGLE_BTP_WIDTH;
const int ExtrinsicROS::LASER_HEIGHT = 135 / ANGLE_BTP_HEIGHT;
const int ExtrinsicROS::IMAGES_PER_TURN = 16;
const double ExtrinsicROS::TOLERANCE_ANGLE = 0.025; // 1,43 Grad
const int ExtrinsicROS::POINTS_FOR_CALIBRATION = 3;
const bool ExtrinsicROS::LEFT_TURN = false;
const int ExtrinsicROS::INTENSITY_START = 200;
const int ExtrinsicROS::INTENSITY_END = 700;

template<typename T>
T getParam(ros::NodeHandle& nH, const std::string& name, const T& defaultValue)
{
	T value;
	if (nH.getParam(name, value))
	{
		ROS_INFO_STREAM("Found parameter: " << name << ", value: " << value);
		return value;
	}
	else
		ROS_WARN_STREAM("Cannot find value for parameter: " << name << ", assigning default: " << defaultValue);
	return defaultValue;
}

ExtrinsicROS::ExtrinsicROS()
{
	ros::NodeHandle privNh ("~");

	// Hole die Namen der Koordinatensysteme.
	baseFrame = getParam<string>(privNh, "base_frame", "/world");
	rotaryPlateFrame = getParam<string>(privNh, "rotary_plate_frame", "/rotary_plate");
	cameraFrame = getParam<string>(privNh, "camera_frame", "/thermo_cam");

	// Hole die Kameradrehung.
	camRPY[0] = getParam<double>(privNh, "cam_roll", -M_PI_2);
	camRPY[1] = getParam<double>(privNh, "cam_pitch", 0);
	camRPY[2] = getParam<double>(privNh, "cam_yaw", M_PI);

	// Hole Schrittgroesse fuer das Verschieben der Kameraposition.
	shiftStep = getParam<double>(privNh, "shift_step", 0.005);

	// Erstelle die Subscriber fuer die Punktwolke, Kamerabilder und fuer die CameraInfo-Nachrichten.
	cloudSub = nH.subscribe("/cloud", 1, &ExtrinsicROS::cloudCallback, this);
	image_transport::ImageTransport it(nH);
	camSub = it.subscribeCamera("/thermo_cam/image_raw", 1, &ExtrinsicROS::imageCallback, this);


	cloudPoints.resize(LASER_HEIGHT);
	for(int i = 0; i < LASER_HEIGHT; ++i)
	{
		cloudPoints[i].resize(LASER_WIDTH);
		for (int j = 0; j < LASER_WIDTH; ++j)
		{
			cloudPoints[i][j] = -1;
		}
	}

	angleBTI = M_PI * 2 / IMAGES_PER_TURN;
	transformsC.resize(IMAGES_PER_TURN);
	imagesC.resize(IMAGES_PER_TURN);
	imageNoC = 0;
	gotFirstImage = false;
	gotCloud = false;
	gotAllImages = false;
	camRotation.setRotation(createQuaternionFromRPY(camRPY[0], camRPY[1], camRPY[2]));
	camRotation.inverse();
}

ExtrinsicROS::~ExtrinsicROS()
{

}

bool ExtrinsicROS::calibrate()
{
	bool isCalibrated = false;
	if (!pointsFromCloud.empty() && !pointsFromImage.empty())
	{
		if (pointsFromCloud.size() != pointsFromImage.size()
			|| (int)pointsFromCloud.size() < POINTS_FOR_CALIBRATION)
		{
			ROS_ERROR("[extrinsic_calibration] The number of image points is not correct!");
			exit(-1);
		}

		Vector3 camPosition(0, 0, 0);
		int direction = left;
		double error;

#ifdef DEBUG
		Mat img (imagesC[imageNoC].clone());
		Transform camTransform(camRotation);

		for (size_t i = 0; i < pointsFromCloud.size(); i++)
		{
			tf::Point tfPoint(pointsFromCloud[i].x, pointsFromCloud[i].y, pointsFromCloud[i].z);
			tf::Point tmpPoint = camTransform * transformsC[imageNoC] * tfPoint;
			pcl::PointXYZ transformedPoint;
			transformedPoint.x = tmpPoint.m_floats[0];
			transformedPoint.y = tmpPoint.m_floats[1];
			transformedPoint.z = tmpPoint.m_floats[2];

			cv::Point3d xyz(transformedPoint.x, transformedPoint.y, transformedPoint.z);
			cv::Point2d point2d;
			point2d = camModel.project3dToPixel(xyz);

			if (point2d.x >= 0 && point2d.x < img.cols
				&& point2d.y >= 0 && point2d.y < img.rows)
			{
				cv::Vec3b &bgrPixel = img.at<cv::Vec3b>(point2d.y, point2d.x);
				bgrPixel.val[2] = (uint32_t) 0; // R
				bgrPixel.val[1] = (uint32_t) 255; // G
				bgrPixel.val[0] = (uint32_t) 0; // B
			}
		}
#endif

		ROS_INFO("[extrinsic_calibration] ------------Calibration------------");
		ROS_INFO("[extrinsic_calibration] Directions: 0..6 (left, right, up, down, forward, backward, none)");
		ROS_INFO("[extrinsic_calibration] Direction: %d", direction);
		while (direction != none)
		{
			Vector3 tmpCamPos(camPosition);
			shiftPosition(tmpCamPos, direction);
			error = calculateError(camPosition, direction);
			double newError = calculateError(tmpCamPos, direction);

			if (newError < error)
			{
				camPosition = tmpCamPos;
				error = newError;
			}
			else
			{
				ROS_INFO("[extrinsic_calibration]     Error: %f", error);
				direction++;
				ROS_INFO("[extrinsic_calibration] Direction: %d", direction);
			}

#ifdef DEBUG
			Mat tmpImage (img.clone());
			for (size_t i = 0; i < pointsFromCloud.size(); i++)
			{
				tf::Point tfPoint(pointsFromCloud[i].x, pointsFromCloud[i].y, pointsFromCloud[i].z);
				camTransform.setOrigin(camPosition);
				tf::Point tmpPoint = camTransform * transformsC[imageNoC] * tfPoint;
				pcl::PointXYZ transformedPoint;
				transformedPoint.x = tmpPoint.m_floats[0];
				transformedPoint.y = tmpPoint.m_floats[1];
				transformedPoint.z = tmpPoint.m_floats[2];

				cv::Point3d xyz(transformedPoint.x, transformedPoint.y, transformedPoint.z);
				cv::Point2d point2d;
				point2d = camModel.project3dToPixel(xyz);

				if (point2d.x >= 0 && point2d.x < tmpImage.cols
					&& point2d.y >= 0 && point2d.y < tmpImage.rows)
				{
					cv::Vec3b &bgrPixel = tmpImage.at<cv::Vec3b>(point2d.y, point2d.x);
					bgrPixel.val[2] = (uint32_t) 255; // R
					bgrPixel.val[1] = (uint32_t) 255; // G
					bgrPixel.val[0] = (uint32_t) 0; // B
				}
			}
			imshow("Points from Cloud", tmpImage);
			waitKey(500);
#endif
		}

		Transform tmpTrans(camRotation);
		tmpTrans.inverse();
		Vector3 finalCamPos = tmpTrans * camPosition;
		finalCamPos.setX(finalCamPos.getX() * -1);
		finalCamPos.setY(finalCamPos.getY() * -1);
		finalCamPos.setZ(finalCamPos.getZ() * -1);

		ROS_INFO_STREAM("[extrinsic_calibration] -------Calibration complete-------- " << std::endl
						<< "------Camera tf------" << std::endl
						<< "X(" << finalCamPos.getX() << ")" << std::endl
						<< "Y(" << finalCamPos.getY()	<< ")" << std::endl
						<< "Z(" << finalCamPos.getZ() << ")" << std::endl
						<< "Yaw(" << camRPY[2] << ")" << std::endl
						<< "Pitch(" << camRPY[1] << ")" << std::endl
						<< "Roll(" << camRPY[0]  << ")" << std::endl
						<< "frame_id(" << rotaryPlateFrame << ")" << std::endl
						<< "child_frame_id(" << cameraFrame << ")" << std::endl
						<< "---------------------");

		ofstream launchFile;
		string filename = ros::package::getPath("extrinsic_calibration") + "/launch/cam_tf.launch";
		launchFile.open(filename.c_str(), ios::out | ios::trunc );
		if (!launchFile)
			ROS_ERROR_STREAM("[extrinsic_calibration] Could not open file (" << filename << ") to save calibration!");
		else
		{
			launchFile << "<launch>" << endl
					   << "<!-- [Usage: static_transform_publisher x y z yaw pitch roll frame_id child_frame_id period(milliseconds)] -->" << endl
					   << "<node pkg=\"tf\" " << endl
					   << "     type=\"static_transform_publisher\" " << endl
					   << "     name=\"cam_link_broadcaster\" " << endl << fixed << setprecision(4)
					   << "     args=\"" << finalCamPos.getX() << " " << finalCamPos.getY() << " " << finalCamPos.getZ() << " "
					   	   	   	   	     << camRPY[2] << " " << camRPY[1] << " " << camRPY[0] << " "
					   	   	   	   	     << rotaryPlateFrame << " " << cameraFrame << " 10\" />" << endl
					   << "</launch>";
			launchFile.close();
			ROS_INFO_STREAM("[extrinsic_calibration] Saved calibration to file:" << filename);
		}

		isCalibrated = true;
	}

	return isCalibrated;
}

double ExtrinsicROS::calculateError(Vector3 &camPosition, int direction)
{
	Point2f points2DFromCloud[pointsFromCloud.size()];

	for (size_t i = 0; i < pointsFromCloud.size(); i++)
	{
		tf::Point tfPoint(pointsFromCloud[i].x, pointsFromCloud[i].y, pointsFromCloud[i].z);
		Transform tmpTrans(camRotation, camPosition);
		tf::Point tmpPoint = tmpTrans * transformsC[imageNoC] * tfPoint;
		pcl::PointXYZ transformedPoint;
		transformedPoint.x = tmpPoint.m_floats[0];
		transformedPoint.y = tmpPoint.m_floats[1];
		transformedPoint.z = tmpPoint.m_floats[2];

		cv::Point3d xyz(transformedPoint.x, transformedPoint.y, transformedPoint.z);
		points2DFromCloud[i] = camModel.project3dToPixel(xyz);
	}

	double error;

	if (direction == left || direction == right)
	{
		double idealX = (pointsFromImage[1].x - pointsFromImage[0].x) / 2 + pointsFromImage[0].x;
		double actuallyX = (points2DFromCloud[1].x - points2DFromCloud[0].x) / 2 + points2DFromCloud[0].x;
		error = fabs(idealX - actuallyX);
	}
	else if (direction == up || direction == down)
	{
		double idealY = (pointsFromImage[2].y - pointsFromImage[0].y) / 2 + pointsFromImage[0].y;
		double actuallyY = (points2DFromCloud[2].y - points2DFromCloud[0].y) / 2 + points2DFromCloud[0].y;
		error = fabs(idealY - actuallyY);
	}
	else if (direction == forward || direction == backward)
	{
		Point2f vec = pointsFromImage[2] - pointsFromImage[1];
		double idealL = sqrt(vec.x * vec.x + vec.y * vec.y);
		vec = points2DFromCloud[2] - points2DFromCloud[1];
		double actuallyL = sqrt(vec.x * vec.x + vec.y * vec.y);
		error = fabs(idealL - actuallyL);
	}
	else
	{
		ROS_ERROR("[extrinsic_calibration] calculateError(Vector3 &, int) got incorrect direction!");
		error = -1;
	}

	return error;
}

void  ExtrinsicROS::shiftPosition(Vector3 &position, int direction)
{
	switch(direction)
	{
	case left:
		position.setX(position.getX() - shiftStep);
		break;
	case right:
		position.setX(position.getX() + shiftStep);
		break;
	case up:
		position.setY(position.getY() - shiftStep);
		break;
	case down:
		position.setY(position.getY() + shiftStep);
		break;
	case forward:
		position.setZ(position.getZ() + shiftStep);
		break;
	case backward:
		position.setZ(position.getZ() - shiftStep);
		break;
	default:
		ROS_ERROR("[extrinsic_calibration] shiftPosition(Vector3 &, int) got incorrect direction!");
		break;
	}
}

void ExtrinsicROS::addPointsLS(std::vector<cv::Point2f> points)
{
	pointsFromCloud.resize(points.size());

	for (size_t i = 0; i < points.size(); ++i)
	{
		int noOfPoint = cloudPoints[points[i].y][points[i].x];
		if (noOfPoint == -1)
			noOfPoint = findNextPoint(points[i]);

		if (noOfPoint >= 0 && (uint)noOfPoint < cloud.points.size())
		{
			pointsFromCloud[i] = pcl::PointXYZ(cloud.points[noOfPoint].x,
											   cloud.points[noOfPoint].y,
											   cloud.points[noOfPoint].z);
		}

	}
}

void ExtrinsicROS::addPointsC(std::vector<cv::Point2f> points)
{
	pointsFromImage = points;
}

void ExtrinsicROS::nextImageC()
{
	imageNoC--;
	imageNoC = (imageNoC < 0)? IMAGES_PER_TURN - 1: imageNoC;
	Q_EMIT gotImageC(imagesC[imageNoC]);
}

void ExtrinsicROS::prevImageC()
{
	imageNoC++;
	imageNoC %= IMAGES_PER_TURN;
	Q_EMIT gotImageC(imagesC[imageNoC]);
}

int ExtrinsicROS::findNextPoint(Point2i point)
{
	std::deque<Point2i> neighbors;
	int noOfPoint = -1;
	addNeighbors(point, &neighbors);

	// Jedes Pixel hat 8 Nachbarn und bis zu den 3. Nachbarn wird gesucht, d.h. 8*8*8=512.
	for (int i = 0; noOfPoint == -1 && !neighbors.empty() && i < 512; ++i)
	{
		if (neighbors.front().y >= 0 || neighbors.front().y < LASER_HEIGHT
			|| neighbors.front().x >= 0 || neighbors.front().x < LASER_WIDTH)
		{
			noOfPoint = cloudPoints[neighbors.front().y][neighbors.front().x];
			addNeighbors(neighbors.front(), &neighbors);
		}
		neighbors.pop_front();
	}

	return noOfPoint;
}

void ExtrinsicROS::addNeighbors(Point2i &point, std::deque<Point2i> *neighbors)
{
	neighbors->push_back(Point2i(point.x - 1, point.y - 1));
	neighbors->push_back(Point2i(point.x    , point.y - 1));
	neighbors->push_back(Point2i(point.x + 1, point.y - 1));
	neighbors->push_back(Point2i(point.x + 1, point.y));
	neighbors->push_back(Point2i(point.x + 1, point.y + 1));
	neighbors->push_back(Point2i(point.x    , point.y + 1));
	neighbors->push_back(Point2i(point.x - 1, point.y + 1));
	neighbors->push_back(Point2i(point.x - 1, point.y));
}

void ExtrinsicROS::createImageFromCloud(cv::Mat *image)
{
	for (int i = 0; i < image->rows; ++i)
		for (int j = 0; j < image->cols; ++j)
			image->at<uchar>(i, j) = 0;

	for (size_t i = 0; i < cloud.points.size (); ++i)
	{
		int angleZ = atan2(cloud.points[i].y, cloud.points[i].x) * 180 / M_PI / ANGLE_BTP_WIDTH;
		int angleXY = atan2(cloud.points[i].z, sqrt(cloud.points[i].x * cloud.points[i].x
												   + cloud.points[i].y * cloud.points[i].y))
												* 180 / M_PI / ANGLE_BTP_HEIGHT;

		// angleZ wird Werte von -180 bis +180 haben. posX erhaelt durch diese Umwandlung
		// nur Werte von 0 bis 360 oder hoeher, abhaengig von ANGLE_BTP_WIDTH.
		int posX = abs(angleZ - 180 / ANGLE_BTP_WIDTH);

		// angleXY wird Werte von -45 bis 90 haben. posY erhaelt durch diese Umwandlung
		// nur Werte von 0 bis 135 oder hoeher, abhaengig von ANGLE_BTP_HEIGHT.
		int posY = abs(angleXY - 90 / ANGLE_BTP_HEIGHT);

		cloudPoints[posY][posX] = i;
		int intensity = cloud.points[i].intensity;
		intensity = (intensity < INTENSITY_START)? INTENSITY_START: intensity;
		intensity = (intensity > INTENSITY_END)? INTENSITY_END: intensity;
		intensity -= INTENSITY_START;
		intensity = (double)intensity / (INTENSITY_END - INTENSITY_START) * 255.;
		image->at<uchar>(posY, posX) = intensity;
	}
}

double ExtrinsicROS::getAngle(tf::Transform &transform)
{
	double angle;
	geometry_msgs::Quaternion rot;

	tf::quaternionTFToMsg(transform.getRotation(), rot);
	angle = 2.*atan2(rot.y, rot.w) + M_PI_2;
	if (angle>M_PI)
		angle -= 2.*M_PI;
	else if (angle<-M_PI)
		angle += 2.*M_PI;

	return angle;
}

void ExtrinsicROS::imageCallback(const sensor_msgs::ImageConstPtr &imgMsgP, const sensor_msgs::CameraInfoConstPtr& infoMsgP)
{
	tf::StampedTransform transform;
	static int i = 0;
	double zAngle;

	if (!gotAllImages)
	{
		try
		{
			if (!tfListener.waitForTransform(baseFrame, rotaryPlateFrame, imgMsgP->header.stamp, ros::Duration(1.)))
				ROS_WARN_STREAM("[extrinsic_calibration] Timeout (1s) while waiting between "<<rotaryPlateFrame<<
								" and "<<baseFrame<<".");

			tfListener.lookupTransform(rotaryPlateFrame, baseFrame, imgMsgP->header.stamp, transform);
			Transform tmpTrans(camRotation);
			tmpTrans *= transform;
			zAngle = getAngle(tmpTrans);
		}
		catch (const tf::ExtrapolationException & e) {
			ROS_ERROR_STREAM("[extrinsic_calibration] Could not resolve rotating angle of the rotary plate. " << e.what());
			ROS_INFO("[extrinsic_calibration] This is normal, if the rotary plate has not yet reached the start position!");
			return;
		}

		if (!gotFirstImage && zAngle > -TOLERANCE_ANGLE && zAngle < TOLERANCE_ANGLE)
		{
			cv_bridge::CvImagePtr cvImage = cv_bridge::toCvCopy(*imgMsgP, "bgr8");
			transformsC[0] = transform;
			camModel.fromCameraInfo(infoMsgP);
			camModel.rectifyImage(cvImage->image, imagesC[0]);
			gotFirstImage = true;
			ROS_INFO_STREAM("[extrinsic_calibration] "<< i <<". image at angle:" << zAngle * 180 / M_PI);
			if (LEFT_TURN)
				i = 1;
			else
				i = IMAGES_PER_TURN - 1;
		}
		else if (gotFirstImage)
		{
			double zAngleP = (zAngle < 0)? zAngle + 2 * M_PI: zAngle;

			if ((zAngleP > i * angleBTI - TOLERANCE_ANGLE) && (zAngleP < i * angleBTI + TOLERANCE_ANGLE))
			{
				cv_bridge::CvImagePtr cvImage = cv_bridge::toCvCopy(*imgMsgP, "bgr8");
				camModel.rectifyImage(cvImage->image, imagesC[i]);
				transformsC[i] = transform;
				ROS_INFO_STREAM("[extrinsic_calibration] "<< i <<". image at angle:" << zAngle * 180 / M_PI);
				if (LEFT_TURN)
					i++;
				else
					i--;
			}

			if (LEFT_TURN? i == IMAGES_PER_TURN: i == 0)
			{
				gotAllImages = true;
				gotFirstImage = false;
				Q_EMIT gotImageC(imagesC[imageNoC]);
			}
		}
	}
}

void ExtrinsicROS::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloudMsgP)
{
	static bool first = true;

	if (!first && !gotCloud)
	{
		pcl::fromROSMsg(*cloudMsgP,cloud);
		gotCloud = true;
		cv::Mat image(LASER_HEIGHT, LASER_WIDTH, CV_8UC1);
		createImageFromCloud(&image);
		Q_EMIT gotImageLS(image);
	}
	first = false;
}
