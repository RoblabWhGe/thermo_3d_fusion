/*
 * beispielGUI.cpp
 *
 *  Created on: 10.12.2012
 *      Author: Tom-M. Liebelt
 */

#include "beispielGUI.h"

beispielGUI::beispielGUI()
{
	widget.setWindowTitle("Kamerabild");
	widget.setGeometry(QRect(0, 0, 440, 240));
	imgLabel = new QLabel(&widget);
	imgLabel->setGeometry(QRect(0, 0, 320, 240));

	imgButton = new QPushButton(&widget);
	imgButton->setGeometry(QRect(330, 110, 100, 20));
	imgButton->setText("Bild");
	widget.show();
	QObject::connect(imgButton, SIGNAL(clicked()), this, SIGNAL(nextImage()));
}


void beispielGUI::showImage(cv::Mat image)
{
	if (!image.empty())
	{
		cv::Mat dest;
		cv::cvtColor(image, dest,CV_BGR2RGB);
		QImage qImage((uchar*)dest.data, dest.cols, dest.rows,QImage::Format_RGB888);
		imgLabel->setPixmap(QPixmap::fromImage(qImage));
	}
}
