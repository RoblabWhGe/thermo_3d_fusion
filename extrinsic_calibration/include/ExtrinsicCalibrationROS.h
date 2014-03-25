/*
 * ExtrinsicCalibrationROS.h
 *
 *  Created on: 11.01.2013
 *      Author: Tom-M. Liebelt
 */

#ifndef EXTRINSICCALIBRATIONROS_H_
#define EXTRINSICCALIBRATIONROS_H_

//#define DEBUG

#include <ros/ros.h>
#include <ros/package.h>
#include <QObject>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <deque>
#include <fstream>
#ifdef DEBUG
#include <opencv2/highgui/highgui.hpp>
#endif

class ExtrinsicROS : public QObject
{
	Q_OBJECT

public:
	/** Winkel zwischen zwei Punkte des Laserscanners in der Breite. */
	static const double ANGLE_BTP_WIDTH;

	/** Winkel zwischen zwei Punkte des Laserscanners in der Hoehe. */
	static const double ANGLE_BTP_HEIGHT;

	/** Anzahl der Laserscannerpunkte in der Breite. */
	static const int LASER_WIDTH;

	/** Anzahl der Laserscannerpunkte in der Hoehe. */
	static const int LASER_HEIGHT;

	/** Die Toleranz fuer die Aufnahmepositionen der Kamera. */
	static const double TOLERANCE_ANGLE;

	/** Anzahl der aufzunehmenden Bilder pro Umdrehung. */
	static const int IMAGES_PER_TURN;

	/** Anzahl der fuer die Kalibrierung benoetigten Punkte. */
	static const int POINTS_FOR_CALIBRATION;

	/** Intensitaetsbereich der Remissionswerte fuer die Umwandlung in ein Graustufenbild. */
	static const int INTENSITY_START, INTENSITY_END;

	/**
	 * Die Drehrichtung des Laserscanners / der Kamera.
	 * true, wenn die Drehung gegen den Uhrzeigersinn ablaeuft.
	 */
	static const bool LEFT_TURN;


	ExtrinsicROS();
	~ExtrinsicROS();

	/**
	 * Berechnet die extrinsischen Parameter und speichert sie in eine launch-Datei
	 * (/launch/cam_tf.launch), wenn das Kalibrierungsmuster in der Punktwolke und
	 * im Kamerabild markiert wurde.
	 * Die Punkte muessen dazu in pointsFromImage und pointsFromCloud gespeichert sein.
	 *
	 * @return - true, wenn Kalibrierung erfolgreich
	 */
	bool calibrate();

Q_SIGNALS:
	/**
	 * SIGNAL: Wird gesendet, wenn die zweite Punktwolke empfangen und umgewandelt wurde.
	 *
	 * @param image - die Laserscanner-Daten als 2D-Bild
	 */
	void gotImageLS(cv::Mat image);

	/**
	 * SIGNAL: Wird gesendet, wenn eine 360Grad Drehung erfolgt ist und nextImageC oder prevImageC
	 * aufgerufen wird.
	 *
	 * @param image - das Bild der Kamera
	 */
	void gotImageC(cv::Mat image);

public Q_SLOTS:
	/**
	 * SLOT: Ermittelt aus den 2D-Punkten vom Laserscanner-Bild (Markierung des Kalibrierungsmuster)
	 * die zugehoerigen 3D-Punkte aus der Wolke und speichert sie in pointsFromCloud.
	 *
	 * @param - die Markierungen des Kalibrierungsmusters in dem von gotImageLS gesendetem Bild
	 */
	void addPointsLS(std::vector<cv::Point2f> points);

	/**
	 * SLOT: Speichert die 2D-Punkte vom Kamera-Bild (Markierung des Kalibrierungsmuster)
	 * in pointsFromImage.
	 *
	 * @param - die Markierungen des Kalibrierungsmusters in dem von gotImageC gesendetem Bild
	 */
	void addPointsC(std::vector<cv::Point2f> points);

	/**
	 * SLOT: Gibt das naechste Kamerabild ueber gotImageC zurueck.
	 */
	void nextImageC();

	/**
	 * SLOT: Gibt das vorherige Kamerabild ueber gotImageC zurueck.
	 */
	void prevImageC();

private:

	ros::NodeHandle nH;

	/** Subscriber fuer die Punktwolke. **/
	ros::Subscriber cloudSub;

	/** Subscriber fuer das Kamerabild und die Camera Info Nachricht. */
	image_transport::CameraSubscriber camSub;

	/** Punktwolke vom 3D-Laserscanner. */
	pcl::PointCloud<pcl::PointXYZI> cloud;

	/** 2D-Vector um eine Zuordnung vom Pixel zum 3D-Punkt herzustellen. */
	std::vector<std::vector<int> > cloudPoints;

	/** Transformlistener fuer die Position der Kamera. */
	tf::TransformListener tfListener;

	/** Nummer des ausgewaehlten Kamerabildes. */
	int imageNoC;

	/** Die aufgenommenen Kamerabilder. */
	std::vector<cv::Mat> imagesC;

	/** Die zu den Kameraaufnahmen passenden rotary_plate-Transformationen. */
	std::vector<tf::StampedTransform> transformsC;

	/** Die Drehung des Kamerakoordinatensystems im rotary_plate-KS als Quaternion.*/
	tf::Matrix3x3 camRotation;

	/** Die Drehung des Kamerakoordinatensystems im rotary_plate-KS in Roll, Pitch, Yaw.*/
	double camRPY[3];

	/** Winkel zwischen zwei Bildern. */
	double angleBTI;

	/** Eckpunkte des Kalibrierungsmusters auf dem Kamerabild. */
	std::vector<cv::Point2f> pointsFromImage;

	/** Eckpunkte des Kalibrierungsmusters in der Punktwolke. */
	std::vector<pcl::PointXYZ> pointsFromCloud;

	/** Kameramodell der Thermalkamera. */
	image_geometry::PinholeCameraModel camModel;

	/** Die Namen der Koordinatensysteme. */
	std::string baseFrame, rotaryPlateFrame, cameraFrame;

	/** Die Schrittgroesse fuer das Verschieben der Kamera. */
	double shiftStep;

	bool gotCloud, gotFirstImage, gotAllImages;

	int Direction;
	enum {left, right, up, down, forward, backward, none};


	/**
	 * Sucht in cloudPoints in der naeheren Umgebung des uebergebenen Punktes einen 2D-Punkt,
	 * der auf einen 3D-Punkt der Punktwolke verweist.
	 *
	 * @param point - der 2D-Punkt im von gotImageLS erzeugtem Bild
	 * @return - die Nummer des 3D-Punktes, wenn einer gefunden wurde, ansonsten -1
	 */
	int findNextPoint(cv::Point2i point);

	/**
	 * Fuegt die 8-Nachbarn eines Pixels der uebergebenen deque hinzu.
	 *
	 * @param point - Pixel
	 * @param neighbors - die deque der Nachbarn
	 */
	void addNeighbors(cv::Point2i &point, std::deque<cv::Point2i> *neighbors);

	/**
	 * Erzeugt ein 2D-Bild aus der gesamten Punktwolke.
	 *
	 * @param image - Zeiger auf eine zuvor erzeugte Mat, mit der Groesse: LASER_WIDTH x LASER_HEIGHT
	 */
	void createImageFromCloud(cv::Mat *image);

	/**
	 * Gibt den Pitch-Drehwinkel der Thermalkamera zurueck.
	 *
	 * @param transform - die Transformation von dem der Winkel bestimmt wird
	 * @return - der Drehwinkel
	 */
	double getAngle(tf::Transform &transform);

	/**
	 * Berechnet den Fehler, mit dem festgestellt werden kann ob die neue Kameraposition
	 * besser oder schlechter ist als die alte.
	 * Es muss die Richtung, in die die Kamera verschoben wurde, angegeben werden, da
	 * fuer die Richtungen links/rechts , hoch/runter und vor/zurueck jeweils eine eigene
	 * Fehlerberechnung existiert.
	 *
	 * @param camPosition - die neue Kameraposition
	 * @param direction - die Richtung in die die Kamera geschoben wurde
	 * @return - der berechnete Fehler
	 */
	double calculateError(tf::Vector3 &camPosition, int direction);

	/**
	 * Verschiebt die uebergebenen Kameraposition in die uebergebene Richtung um einen shiftStep.
	 *
	 * @param position - die Kameraposition
	 * @param direction - die Richtung in die geschoben werden soll
	 */
	void shiftPosition(tf::Vector3 &position, int direction);

	/**
	 * Callback-Funktion fuer die Thermalkamera- und CameraInfo-Nachrichten.
	 *
	 * @param imgMsgP - Kameranachricht
	 * @param infoMsgP - CameraInfo-Nachricht
	 */
	void imageCallback(const sensor_msgs::ImageConstPtr &imgMsgP, const sensor_msgs::CameraInfoConstPtr& infoMsgP);

	/**
	 * Callback-Funktion fuer die Laserscanner-Punktwolke.
	 *
	 * @param cloudMsgP - Punktwolke
	 */
	void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &cloudMsgP);

};


#endif /* EXTRINSICCALIBRATION_H_ */
