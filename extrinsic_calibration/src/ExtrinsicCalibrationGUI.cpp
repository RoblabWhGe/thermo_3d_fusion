#include "ExtrinsicCalibrationGUI.h"

bool ClickFilter::eventFilter( QObject* watched, QEvent* event )
{
	if (event->type() == QEvent::MouseButtonPress)
	{
		const QMouseEvent* const me = static_cast<const QMouseEvent*>( event );
		Q_EMIT clicked(me->pos());

		return true;
	}

    return false;
}

//-----------------------------------------------------------------------

const int ExtrinsicGUI::CORNERS_PER_PATTERN = 3;
const int ExtrinsicGUI::SCALE_FACTOR = 4;

ExtrinsicGUI::ExtrinsicGUI()
{

	//--------------Laserscanner GUI-------------------------
	widgetLS.setWindowTitle("Laserscan");
	widgetLS.setGeometry(QRect(10, 10, 1440, 600));
	imgLabelLS = new QLabel(&widgetLS);
	imgLabelLS->setGeometry(QRect(0, 0, 1440, 540));

	okButtonLS = new QPushButton(&widgetLS);
	okButtonLS->setGeometry(QRect(10, 560, 160, 20));
	okButtonLS->setText("Ok");
	okButtonLS->setEnabled(false);

	clearButtonLS = new QPushButton(&widgetLS);
	clearButtonLS->setGeometry(QRect(180, 560, 160, 20));
	clearButtonLS->setText("Clear");

	imgLabelLS->installEventFilter(&watcherLS);
	QObject::connect(&watcherLS, SIGNAL(clicked(QPoint)), this, SLOT(savePosLS(QPoint)));
	QObject::connect(okButtonLS, SIGNAL(clicked()), this, SLOT(sendPointsLS()));
	QObject::connect(clearButtonLS, SIGNAL(clicked()), this, SLOT(clearImageLS()));

	pointNoLS = 0;

	//--------------Camera GUI-------------------------------
	widgetC.setWindowTitle("Camera images");
	widgetC.setGeometry(QRect(100, 400, 820, 480));
	imgLabelC = new QLabel(&widgetC);
	imgLabelC->setGeometry(QRect(0, 0, 640, 480));

	okButtonC = new QPushButton(&widgetC);
	okButtonC->setGeometry(QRect(650, 15, 160, 50));
	okButtonC->setText("Ok");
	okButtonC->setEnabled(false);

	clearButtonC = new QPushButton(&widgetC);
	clearButtonC->setGeometry(QRect(650, 80, 160, 50));
	clearButtonC->setText("Clear");

	nextButtonC = new QPushButton(&widgetC);
	nextButtonC->setGeometry(QRect(650, 145, 160, 50));
	nextButtonC->setText("Next ->");

	prevButtonC = new QPushButton(&widgetC);
	prevButtonC->setGeometry(QRect(650, 210, 160, 50));
	prevButtonC->setText("<- Prev");

	imgLabelC->installEventFilter(&watcherC);
	QObject::connect(&watcherC, SIGNAL(clicked(QPoint)), this, SLOT(savePosC(QPoint)));
	QObject::connect(okButtonC, SIGNAL(clicked()), this, SLOT(sendPointsC()));
	QObject::connect(clearButtonC, SIGNAL(clicked()), this, SLOT(clearImageC()));
	QObject::connect(nextButtonC, SIGNAL(clicked()), this, SIGNAL(nextImageC()));
	QObject::connect(prevButtonC, SIGNAL(clicked()), this, SIGNAL(prevImageC()));

	pointNoC = 0;

}


void ExtrinsicGUI::setImageLS(cv::Mat image)
{
	cv::Mat dest;
	cv::cvtColor(image, dest, CV_GRAY2RGB);
	QImage qImage((uchar*)dest.data, dest.cols, dest.rows,QImage::Format_RGB888);
	modPixLS = QPixmap::fromImage(qImage);
	imgLabelLS->setPixmap(modPixLS);
	origPixLS = modPixLS;
	widgetLS.show();
}

void ExtrinsicGUI::drawPointsLS()
{
	QPainter p( &modPixLS );
	p.setBrush(Qt::green);
	for (size_t i = 0; i < cornersLS.size(); ++i)
		p.drawEllipse(QPoint(cornersLS[i].x(), cornersLS[i].y()), 3, 3);
	p.end();
	imgLabelLS->setPixmap(modPixLS);
}

void ExtrinsicGUI::savePosLS(QPoint point)
{
	if (pointNoLS == CORNERS_PER_PATTERN - 1)
			okButtonLS->setEnabled(true);

	if (pointNoLS < CORNERS_PER_PATTERN)
	{
		cornersLS.push_back(QPoint(point.x(), point.y()));
		drawPointsLS();
		pointNoLS++;
	}
}

void ExtrinsicGUI::sendPointsLS()
{
	std::vector<cv::Point2f> points;
	for (size_t i = 0; i < cornersLS.size(); ++i)
		points.push_back(cv::Point2f(cornersLS[i].x(), cornersLS[i].y()));
	Q_EMIT gotAllCornersLS(points);

#ifdef DEBUG
	okButtonLS->setEnabled(false);
#else
	widgetLS.close();
#endif
}

void ExtrinsicGUI::clearImageLS()
{
	okButtonLS->setEnabled(false);
	cornersLS.clear();
	pointNoLS = 0;
	modPixLS = origPixLS;
	imgLabelLS->setPixmap(origPixLS);
}

void ExtrinsicGUI::setImageC(cv::Mat image)
{
	cv::Mat resizeI, dest;
	cv::resize(image, resizeI, cv::Size(), SCALE_FACTOR, SCALE_FACTOR, cv::INTER_LINEAR);
	cv::cvtColor(resizeI, dest, CV_BGR2RGB);
	QImage qImage((uchar*)dest.data, dest.cols, dest.rows,QImage::Format_RGB888);
	modPixC = QPixmap::fromImage(qImage);
	imgLabelC->setPixmap(modPixC);
	origPixC = modPixC;
	okButtonC->setEnabled(false);
	cornersC.clear();
	pointNoC = 0;
	widgetC.show();
}

void ExtrinsicGUI::drawPointsC()
{
	QPainter p( &modPixC );
	p.setBrush(Qt::green);
	for (size_t i = 0; i < cornersC.size(); ++i)
		p.drawEllipse(QPoint(cornersC[i].x() * SCALE_FACTOR, cornersC[i].y() * SCALE_FACTOR), 3, 3);
	p.end();
	imgLabelC->setPixmap(modPixC);
}

void ExtrinsicGUI::savePosC(QPoint point)
{
	if (pointNoC == CORNERS_PER_PATTERN - 1)
			okButtonC->setEnabled(true);

	if (pointNoC < CORNERS_PER_PATTERN)
	{
		cornersC.push_back(QPoint(point.x() / SCALE_FACTOR, point.y() / SCALE_FACTOR));
		drawPointsC();
		pointNoC++;
	}
}

void ExtrinsicGUI::sendPointsC()
{
	std::vector<cv::Point2f> points;
	for (size_t i = 0; i < cornersC.size(); ++i)
		points.push_back(cv::Point2f(cornersC[i].x(), cornersC[i].y()));
	Q_EMIT gotAllCornersC(points);

#ifdef DEBUG
	okButtonC->setEnabled(false);
#else
	widgetC.close();
#endif
}

void ExtrinsicGUI::clearImageC()
{
	okButtonC->setEnabled(false);
	cornersC.clear();
	pointNoC = 0;
	modPixC = origPixC;
	imgLabelC->setPixmap(origPixC);
}
