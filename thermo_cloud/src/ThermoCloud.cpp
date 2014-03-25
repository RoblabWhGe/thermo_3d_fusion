/*
 * ThermoCloud.cpp
 *
 *  Created on: 10.01.2013
 *      Author: Tom-M. Liebelt
 */

#include "ThermoCloud.h"

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

ThermoCloud::ThermoCloud():
		imageSub(nH, "/thermo_cam/image_raw", 1),
		infoSub(nH, "/thermo_cam/camera_info", 1),
		tempSub(nH, "/thermo_cam/thermo_data", 1),
		synchronizer(syncPolicy(10),imageSub, infoSub, tempSub)
{
	cloudSub = nH.subscribe("/cloud", 1, &ThermoCloud::cloudCallback, this);
	cloudPub = nH.advertise<pcl::PointCloud<pcl::PointXYZRGB> >("/3d_thermo/rgb_cloud", 1);
	tempPub = nH.advertise<thermo_cloud::ThermoData>("/3d_thermo/thermo_data", 1);
	synchronizer.registerCallback(boost::bind(&ThermoCloud::camCallback, this, _1, _2, _3));

	ros::NodeHandle privNh ("~");

	baseFrame = getParam<std::string>(privNh, "base_frame", "/world");
	cameraFrame = getParam<std::string>(privNh, "camera_frame", "/thermo_cam");
	imagesPerTurn = getParam<int>(privNh, "images_per_turn", 16);
	toleranceAngle = getParam<double>(privNh, "tolerance_angle", 0.025);
	leftTurn = getParam<bool>(privNh, "left_turn", false);
	cloudConfig = getParam<int>(privNh, "cloud_config", 2);
	transparency = getParam<double>(privNh, "transparency", 35);
	intensityStart= getParam<int>(privNh, "intensity_start", 0);
	intensityEnd= getParam<int>(privNh, "intensity_end", 1500);

	angleBTI = M_PI * 2 / imagesPerTurn;
	camTransforms.resize(imagesPerTurn);
	camTemps.resize(imagesPerTurn);
	camImages.resize(imagesPerTurn);
	transparency /= 100;
	if (transparency < 0 || transparency > 1)
		transparency = 0.5;

	gotFirstImage = false;
	gotCloud = false;
	gotEnoughImages = false;
}

ThermoCloud::~ThermoCloud()
{

}

void ThermoCloud::camCallback(const sensor_msgs::ImageConstPtr &imgMsgP,
							  const sensor_msgs::CameraInfoConstPtr &infoMsgP,
							  const thermo_cam_2::ThermoDataConstPtr &tempMsgP)
{
	tf::StampedTransform transform;
	static int i = 0;
	double zAngle;

	try
	{
		if (!tfListener.waitForTransform(baseFrame, cameraFrame, imgMsgP->header.stamp, ros::Duration(1.)))
			ROS_WARN_STREAM("[thermo_cloud] Timeout (1s) while waiting between "<<cameraFrame<<
							" and "<<baseFrame<<" before getting camera angle.");

		tfListener.lookupTransform(cameraFrame, baseFrame, imgMsgP->header.stamp, transform);
		zAngle = getAngle(transform);
	}
	catch (const tf::ExtrapolationException & e) {
		ROS_ERROR_STREAM("[thermo_cloud] Could not resolve rotating angle of the camera: " << e.what());
		ROS_INFO("[thermo_cloud] This is normal, if the rotary plate has not yet reached the start position!");
		return;
	}

	if (!gotFirstImage && zAngle > -toleranceAngle && zAngle < toleranceAngle)
	{
		camModel.fromCameraInfo(infoMsgP);
		saveData(transform, imgMsgP, tempMsgP, 0);
		gotFirstImage = true;
		ROS_DEBUG_STREAM("[thermo_cloud] 0. image at angle:" << zAngle * 180 / M_PI);
		if (leftTurn)
			i = 1;
		else
			i = imagesPerTurn - 1;
	}
	else if (gotFirstImage)
	{
		double zAngleP = (zAngle < 0)? zAngle + 2 * M_PI: zAngle;

		if ((zAngleP > i * angleBTI - toleranceAngle) && (zAngleP < i * angleBTI + toleranceAngle))
		{
			saveData(transform, imgMsgP, tempMsgP, i);
			ROS_DEBUG_STREAM("[thermo_cloud] "<< i <<". image at angle:" << zAngle * 180 / M_PI);
			if (leftTurn)
				i++;
			else
				i--;
		}

		if (leftTurn? i == imagesPerTurn: i == 0)
		{
			gotEnoughImages = true;
			gotFirstImage = false;
		}
	}
}


void ThermoCloud::cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloudMsgP)
{
	pcl::fromROSMsg(*cloudMsgP,lastCloud);
	gotCloud = true;
}

void ThermoCloud::saveData(const tf::StampedTransform &transform,
			  	  	  	   const sensor_msgs::ImageConstPtr &imgMsgP,
						   const thermo_cam_2::ThermoDataConstPtr &tempMsgP,
						   int index)
{
	camTransforms[index] = transform;

	// Kamerabild entzerren.
	cv_bridge::CvImagePtr cvImage = cv_bridge::toCvCopy(*imgMsgP, "bgr8");
	camModel.rectifyImage(cvImage->image, camImages[index]);

	// Thermaldaten entzerren.
	cv::Mat thermoDat((int)tempMsgP->height, (int)tempMsgP->width, cv::DataType<float>::type, (uint8_t*)(&(tempMsgP->data[0])));
	cv::Mat rectThermoDat;
	camModel.rectifyImage(thermoDat, rectThermoDat);
	camTemps[index].height = tempMsgP->height;
	camTemps[index].width = tempMsgP->width;
	size_t size = rectThermoDat.step * rectThermoDat.rows;
	camTemps[index].data.resize(size);
	memcpy((char*)(&camTemps[index].data[0]), rectThermoDat.data, size);
}

int ThermoCloud::estimateCamNo(double angle, int transNo)
{
	double camPos;
	double halfAA =  angleBTI / 2 + 2 * toleranceAngle;
	int camNo;

	transNo = (transNo == -1)? (imagesPerTurn - 1): transNo;
	camPos = getAngle(camTransforms[transNo]);

	if (camPos + halfAA > M_PI || camPos - halfAA < -M_PI)
		camNo = transNo;
	else if (angle <= (camPos - halfAA))
		camNo = estimateCamNo(angle, transNo - 1);
	else if (angle > (camPos + halfAA))
		camNo = estimateCamNo(angle, transNo + 1);
	else
		camNo = transNo;

	return camNo;
}

double ThermoCloud::getAngle(tf::StampedTransform &transform)
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

void ThermoCloud::mainFunc()
{
	ros::Rate loop_rate(100);
	while (ros::ok())
	{
		if (gotCloud && gotEnoughImages)
		{
			ROS_DEBUG_STREAM("[thermo_cloud] point cloud + thermal image");

			pcl::PointCloud<pcl::PointXYZRGB> rgbCloud;
			rgbCloud.resize(lastCloud.size());
			thermo_cloud::ThermoData thermoData;
			thermoData.width = lastCloud.size();
			thermoData.height = 0;
			thermoData.data.resize(lastCloud.size());

			for (size_t i = 0; i < lastCloud.size(); ++i)
			{
				pcl::PointXYZI point = lastCloud.points[i];
				rgbCloud.points[i].x = point.x;
				rgbCloud.points[i].y = point.y;
				rgbCloud.points[i].z = point.z;

				// Waehle das Kamerabild, dass ungefaehr bei dem Winkel dieses Punktes aufgenommen wurde.
				int camNo = estimateCamNo(atan2(point.y, point.x));
				bool foundRGB = false;
				bool changedCam = false;

				while (!foundRGB)
				{
					// Transformiere diesen Punkt in das Kamerakoordinatensystem.
					tf::Point tfPoint(point.x, point.y, point.z);
					tf::Point tmpPoint = camTransforms[camNo] * tfPoint;
					pcl::PointXYZ transformedPoint;
					transformedPoint.x = tmpPoint.m_floats[0];
					transformedPoint.y = tmpPoint.m_floats[1];
					transformedPoint.z = tmpPoint.m_floats[2];

					// Projiziere den Punkt auf die Bildebene der Kamera.
					cv::Point3d xyz(transformedPoint.x, transformedPoint.y, transformedPoint.z);
					cv::Point2d point2d;
					point2d = camModel.project3dToPixel(xyz);

					// Wenn der Punkt innerhalb des Kamerabildes liegt, dann setzte die Farbe des 3D-Punktes.
					if (point2d.x >= 0 && point2d.x < camImages[camNo].cols
						&& point2d.y >= 0 && point2d.y < camImages[camNo].rows)
					{
						foundRGB = true;
						cv::Vec3b bgrPixel = camImages[camNo].at<cv::Vec3b>((int)point2d.y, (int)point2d.x);
						cv::Scalar rgb(bgrPixel.val[2], bgrPixel.val[1], bgrPixel.val[0], 1);
						addColorToPC(rgbCloud,i,rgb);

						//Speichere den Temperaturwert in der ThermoData-Nachricht.
						thermoData.data[i] = camTemps[camNo].data[camTemps[camNo].width * (int)point2d.y + (int)point2d.x];
					}

					// Wenn der Punkt ausserhalb des Kamerabildes liegt, dann wechsle
					// zur naechsten, bzw. zur vorherigen Kameraaufnahme.
					else if (!changedCam)
					{
						if (point2d.x < 0)
						{
							camNo++;
							camNo %= imagesPerTurn;
						}
						else if (point2d.x >= camImages[camNo].cols)
						{
							camNo--;
							camNo = (camNo == -1)? (imagesPerTurn - 1): camNo;
						}
						changedCam = true;
					}

					else
					{
						foundRGB = true;
						cv::Scalar none(0, 0, 0, 0);
						addColorToPC(rgbCloud, i, none);
						thermoData.data[i] = -280;
					}
				}


			}

			rgbCloud.header.stamp = lastCloud.header.stamp;
			rgbCloud.header.frame_id = lastCloud.header.frame_id;
			thermoData.header.stamp = lastCloud.header.stamp;
			thermoData.header.frame_id = lastCloud.header.frame_id;

			cloudPub.publish(rgbCloud);
			tempPub.publish(thermoData);
			ROS_INFO_STREAM("[thermo_cloud] Published rgb cloud. Frame ID: " << baseFrame);
			gotCloud = false;
		}

		ros::spinOnce();
		loop_rate.sleep();
	}
}

void ThermoCloud::addColorToPC(pcl::PointCloud<pcl::PointXYZRGB> &rgbCloud, size_t index, cv::Scalar &color)
{
	uint32_t rgb = 0;
	int intensity = 0;

	if (cloudConfig == 0 || cloudConfig == 1 || cloudConfig == 2)
	{
		intensity = lastCloud.points[index].intensity;
		intensity = (intensity < intensityStart)? intensityStart: intensity;
		intensity = (intensity > intensityEnd)? intensityEnd: intensity;
		intensity -= intensityStart;
		intensity = (double)intensity / (intensityEnd - intensityStart) * 255.;
	}

	switch(cloudConfig)
	{
	case 0: // nur Remissionswerte
		rgb = ((uint32_t)intensity << 16 | (uint32_t)intensity << 8 | (uint32_t)intensity);
		break;
	case 1: // Remissionswerte werden durch Falschfarben ersetzt
		if (color.val[3] == 1)
			rgb = ((uint32_t)color.val[0] << 16 | (uint32_t)color.val[1] << 8 | (uint32_t)color.val[2]);
		else
			rgb = ((uint32_t)intensity << 16 | (uint32_t)intensity << 8 | (uint32_t)intensity);
		break;
	case 2: // Remissionswerte werden von Falschfarben ueberlagert
		if (color.val[3] == 1)
		{
			uint32_t r = transparency * intensity + (1 - transparency) * color.val[0];
			uint32_t g = transparency * intensity + (1 - transparency) * color.val[1];
			uint32_t b = transparency * intensity + (1 - transparency) * color.val[2];
			rgb = (r << 16 | g << 8 | b);
		}
		else
			rgb = ((uint32_t)intensity << 16 | (uint32_t)intensity << 8 | (uint32_t)intensity);
		break;
	case 3: // nur Falschfarben, alles andere schwarz
		if (color.val[3] == 1)
			rgb = ((uint32_t)color.val[0] << 16 | (uint32_t)color.val[1] << 8 | (uint32_t)color.val[2]);
		break;
	default: // alles schwarz

		break;
	}
	rgbCloud.points[index].rgb = *reinterpret_cast<float*>(&rgb);
}


int main(int argc, char** argv)
{

	ros::init(argc, argv, "thermo_cloud");

	ThermoCloud instance;
	instance.mainFunc();
}
