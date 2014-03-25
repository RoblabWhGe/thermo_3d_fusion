/*
 * ThermoCloud.h
 *
 *  Created on: 10.01.2013
 *      Author: Tom-M. Liebelt
 */

#ifndef THERMOCLOUD_H_
#define THERMOCLOUD_H_

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <image_geometry/pinhole_camera_model.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <thermo_cloud/ThermoData.h>
#include <thermo_cam_2/ThermoData.h>
#include <pcl_ros/point_cloud.h>

class ThermoCloud
{
public:
	ThermoCloud();
	~ThermoCloud();

	/**
	 * Fusion der 3D-Laserscanner-Daten mit den 2D-Thermalbild-Daten.
	 */
	void mainFunc();

private:

	ros::NodeHandle nH;

	/** Subscriber fuer die Punktwolke. */
	ros::Subscriber cloudSub;

	/** Publisher fuer die RGB-Punktwolke. */
	ros::Publisher cloudPub;

	/** Publisher fuer die Thermaldaten der Punktwolke. */
	ros::Publisher tempPub;

	/** Subscriber fuer das Kamerabild, die Kamerainfo und die Thermaldaten. */
	message_filters::Subscriber<sensor_msgs::Image> imageSub;
	message_filters::Subscriber<sensor_msgs::CameraInfo> infoSub;
	message_filters::Subscriber<thermo_cam_2::ThermoData> tempSub;

	/** Synchronizer um drei Topic mit einer Callback zu erhalten. */
	typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image,
			sensor_msgs::CameraInfo, thermo_cam_2::ThermoData> syncPolicy;
	typedef message_filters::Synchronizer<syncPolicy> mySynchronizer;
	mySynchronizer synchronizer;

	/** Winkel zwischen zwei Bildern. */
	double angleBTI;

	/** Transformlistener fuer die Position der Thermalkamera. */
	tf::TransformListener tfListener;

	/* Die aufgenommenen Thermalbilder und die dazu passenden Transformationen. */
	std::vector<tf::StampedTransform> camTransforms;
	std::vector<thermo_cam_2::ThermoData> camTemps;
	std::vector<cv::Mat> camImages;

	/* Kameramodell der Thermalkamera. */
	image_geometry::PinholeCameraModel camModel;

	/* Punktwolke aus der empfangenen Nachricht. */
	pcl::PointCloud<pcl::PointXYZI> lastCloud;

	/* Namen der benutzen Koordinatensysteme. */
	std::string baseFrame, cameraFrame;

	/** Anzahl der Bilder pro Umdrehung, die Bilder sollten sich deutlich überlappen. */
	int imagesPerTurn;

	/** Die Toleranz fuer die Aufnahmepositionen der Kamera. */
	double toleranceAngle;

	/**
	 * Die Drehrichtung des Laserscanners / der Kamera.
	 * true, wenn die Drehung gegen den Uhrzeigersinn ablaeuft.
	 */
	bool leftTurn;

	/**
	 * Gibt das Aussehen der auszugebenen Punktwolke an.
	 * 0 = nur Remissionswerte aus der empfangenen Punktwolke
	 * 1 = Remissionswerte werden, wo moeglich, durch Falschfarben der Thermalkamera ersetzt
	 * 2 = Remissionswerte werden, wo moeglich, von Falschfarben ueberlagert
	 * 3 = nur Falschfarben der Kamera, alles andere wird schwarz
	 * other = alle Punkte werden schwarz
	 */
	int cloudConfig;

	/**
	 * Gibt den Grad der Transparenz der Falschfarben in der Punktwolke an.
	 * Nur fuer cloudConfig = 2 noetig.
	 */
	double transparency;

	/**
	 * Gibt an welcher Bereich der Remissionswerte aus der empfangenen Punktwolke uebernommen wird.
	 * Fuer cloudConfig 0, 1 und 2 noetig.
	 */
	int intensityStart, intensityEnd;

	bool gotFirstImage, gotEnoughImages, gotCloud;

	/**
	 * Callback-Funktion fuer die Thermalkamera-, CameraInfo- und ThermoData-Nachrichten.
	 *
	 * Ueber IMAGES_PER_TURN kann die Anzahl der aufzunehmenden Bilder festgelegt werden,
	 * die Bilder werden in "camImages" und die dazugehörige Transformationen
	 * in "camTransforms" gespeichert. Die Thermaldaten werden in camTemps abgelegt.
	 *
	 * @param imgMsgP - Kameranachricht
	 * @param infoMsgP - CameraInfo-Nachricht
	 * @param tempMsgP - ThermoData-Nachricht
	 */
	void camCallback(const sensor_msgs::ImageConstPtr &imgMsgP,
					 const sensor_msgs::CameraInfoConstPtr &infoMsgP,
					 const thermo_cam_2::ThermoDataConstPtr &tempMsgP);
	/**
	 * Callback-Funktion fuer die Laserscanner-Punktwolke.
	 *
	 * Jede Punktwolke wird in "lastCloud" gespeichert.
	 *
	 * @param cloudMsgP - Punktwolke
	 */
	void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloudMsgP);

	/**
	 * Speichert die Daten aus dem camCallback.
	 * Die Kamerabilder und Thermodaten werden zuvor noch entzerrt.
	 *
	 * @param transform - die Kameratransformation
	 * @param imgMsgP - die Kameranachricht
	 * @param tempMsgP - ThermoData-Nachricht
	 * @param index - Speicherposition in den Arrays
	 */
	void saveData(const tf::StampedTransform &transform,
				  const sensor_msgs::ImageConstPtr &imgMsgP,
			  	  const thermo_cam_2::ThermoDataConstPtr &tempMsgP,
				  int index);

	/**
	 * Ermittelt fuer den uebergebenen Winkel die passende Kameraaufnahme und gibt
	 * die Nummer der Aufnahme zurueck.
	 *
	 * @param angle - der Winkel des Punktes aus der Punktwolke
	 * @param transNo - aktuelle Nummer der Kameraaufnahme (fuer Rekursion)
	 */
	int estimateCamNo(double angle, int transNo = 0);

	/**
	 * Gibt den Pitch-Drehwinkel der Thermalkamera zurueck.
	 *
	 * @param transform - die Transformation von dem der Winkel bestimmt wird
	 * @return - der Drehwinkel
	 */
	double getAngle(tf::StampedTransform &transform);

	/**
	 * Gibt einem Punkt aus der RGB-Wolke eine Farbe.
	 * Das Verhalten kann mit der Variable cloudConfig beeinflusst werden.
	 *
	 * @param rgbCloud - die RGB-Wolke in der sich der Punkt fuer die Farbzuweisung befindet
	 * @param index - der Index des o.g. Punktes
	 * @param color - der RGB-Wert, uebergebe (0, 0, 0, 0) falls keine Thermaldaten vorliegen
	 */
	void addColorToPC(pcl::PointCloud<pcl::PointXYZRGB> &rgbCloud, size_t index, cv::Scalar &color);
};

#endif /* THERMOCLOUD_H_ */
