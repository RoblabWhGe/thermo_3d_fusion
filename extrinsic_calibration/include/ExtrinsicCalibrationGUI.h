/*
 * ExtrinsicCalibrationGUI.h
 *
 *  Created on: 14.01.2013
 *      Author: Tom-M. Liebelt
 */

#ifndef EXTRINSICCALIBRATIONGUI_H_
#define EXTRINSICCALIBRATIONGUI_H_

//#define DEBUG

#include <QtGui>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <QMouseEvent>

class ClickFilter : public QObject
{
	Q_OBJECT

Q_SIGNALS:
	void clicked(QPoint point);

protected:
	bool eventFilter(QObject *obj, QEvent *event);
};

//-----------------------------------------------------------------------

class ExtrinsicGUI : public QObject
{
	Q_OBJECT

public:
	ExtrinsicGUI();

	/** Die Anzahl der zu markierenden Punkte auf dem Kalibrierungsmuster. */
	static const int CORNERS_PER_PATTERN;

	/** Der Faktor um den das Kamerabild vergroessert wird. */
	static const int SCALE_FACTOR;


Q_SIGNALS:

	/**
	 * SIGNAL: Wird gesendet, wenn die Punkte fuer das Kalibrierungsmuster im
	 * Laserscan-Bild markiert wurden.
	 *
	 * @param points - die markierten Punkte
	 */
	void gotAllCornersLS(std::vector<cv::Point2f> points);

	/**
	 * SIGNAL: Wird gesendet, wenn die Punkte fuer das Kalibrierungsmuster im
	 * Kamera-Bild markiert wurden.
	 *
	 * @param points - die markierten Punkte
	 */
	void gotAllCornersC(std::vector<cv::Point2f> points);

	/**
	 * SIGNAL: Wird gesendet, wenn das naechste Kamerabild benoetigt wird.
	 */
	void nextImageC();

	/**
	 * SIGNAL: Wird gesendet, wenn das vorherige Kamerabild benoetigt wird.
	 */
	void prevImageC();

public Q_SLOTS:

	/**
	 * SLOT: Wandelt das uebergebene Laserscan-Bild in ein QT-Format um und zeigt es in der GUI an.
	 *
	 * @param image - das Bild im CV-Format
	 */
	void setImageLS(cv::Mat image);

	/**
	 * SLOT: Speichert den uebergebenen Punkt in cornersLS und ruft drawPointsLS auf.
	 *
	 * @param point - der Punkt auf dem Laserscan-Bild
	 */
	void savePosLS(QPoint point);

	/**
	 * Wandelt die Punkte in cornersLS in ein CV-Format um und ruft gotAllCornersLS mit
	 * diesen Punkten auf.
	 */
	void sendPointsLS();

	/**
	 * Loescht alle Punkte aus cornersLS.
	 */
	void clearImageLS();

	/**
	 * SLOT: Wandelt das uebergebene Kamera-Bild in ein QT-Format um und zeigt es in der GUI an.
	 *
	 * @param image - das Bild im CV-Format
	 */
	void setImageC(cv::Mat image);

	/**
	 * SLOT: Speichert den uebergebenen Punkt in cornersC und ruft drawPointsC auf.
	 *
	 * @param point - der Punkt auf dem Kamera-Bild
	 */
	void savePosC(QPoint point);

	/**
	 * Wandelt die Punkte in cornersC in ein CV-Format um und ruft gotAllCornersC mit
	 * diesen Punkten auf.
	 */
	void sendPointsC();

	/**
	 * Loescht alle Punkte aus cornersC.
	 */
	void clearImageC();

private:

	// ----------GUI Komponenten ----------
	ClickFilter watcherLS, watcherC;
	QWidget widgetLS, widgetC;
	QLabel *imgLabelLS, *imgLabelC;
	QPushButton *okButtonLS, *okButtonC, *clearButtonLS, *clearButtonC, *nextButtonC, *prevButtonC;
	QPixmap modPixLS, origPixLS, modPixC, origPixC;
	// ------------------------------------

	/** Anzahl der markierten Punkte im Laserscan-Bild. */
	int pointNoLS;

	/** Anzahl der markierten Punkte im Kamera-Bild. */
	int pointNoC;

	/** Die markierten Punkte im Laserscan-Bild. */
	std::vector<QPoint> cornersLS;

	/** Die markierten Punkte im Kamera-Bild. */
	std::vector<QPoint> cornersC;

	/** Zeige die Punkte aus cornersLS in der GUI an. */
	void drawPointsLS();

	/** Zeige die Punkte aus cornersC in der GUI an. */
	void drawPointsC();
};

#endif /* EXTRINSICCALIBRATIONGUI_H_ */
