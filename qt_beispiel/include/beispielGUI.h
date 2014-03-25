/*
 * beispielGUI.h
 *
 *  Created on: 10.12.2012
 *      Author: Tom-M. Liebelt
 */

#ifndef BEISPIELGUI_H_
#define BEISPIELGUI_H_

#include <QtGui>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

class beispielGUI : public QObject
{
	Q_OBJECT

public:
	beispielGUI();

signals:
	void nextImage();

public slots:
	void showImage(cv::Mat image);

private:
	QWidget widget;
	QLabel *imgLabel;
	QPushButton *imgButton;
};

#endif /* BEISPIELGUI_H_ */
