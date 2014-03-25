/*
 * ViewerGUI.cpp
 *
 *  Created on: 20.02.2013
 *      Author: Tom-M. Liebelt
 */

#include <QtGui>
#include <pcl/visualization/pcl_visualizer.h>

class ViewerGUI : public QObject
{
	Q_OBJECT

public:
	ViewerGUI() :
		viewer ("3D Thermo Viewer"),
		cloud (new pcl::PointCloud<pcl::PointXYZRGB>())
	{
		widget.setWindowTitle("Viewer Control");
		widget.setGeometry(QRect(100, 100, 200, 200));
		startTempL = new QLabel(&widget);
		startTempL->setGeometry(QRect(10, 10, 140, 30));
		startTempL->setText(QString("Start temp. in ") + QChar(0x00B0) + QString("C:"));
		startTemp = new QLineEdit(&widget);
		startTemp->setGeometry(QRect(150, 10, 40, 30));
		endTempL = new QLabel(&widget);
		endTempL->setGeometry(QRect(10, 50, 140, 30));
		endTempL->setText(QString("End temp. in ") + QChar(0x00B0) + QString("C:"));
		endTemp = new QLineEdit(&widget);
		endTemp->setGeometry(QRect(150, 50, 40, 30));
		note = new QLabel(&widget);
		note->setGeometry(QRect(10, 90, 180, 40));
		note->setText("Select the temperature <br />range to be highlighted!");
		okButton = new QPushButton(&widget);
		okButton->setGeometry(QRect(10, 140, 180, 50));
		okButton->setText("Ok");
		QObject::connect(okButton, SIGNAL(clicked()), this, SLOT(setTempInterval()));
		widget.show();

		viewer.setBackgroundColor (0, 0, 0);
		pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
		viewer.addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "thermo_cloud");
		viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "thermo_cloud");
		viewer.addCoordinateSystem (1.0);
		viewer.initCameraParameters ();

		useInterval = false;

	}
	~ViewerGUI(){}

	void pclSpinOnce(int time)
	{
		viewer.spinOnce(time);
	}

private:
	QWidget widget;
	QPushButton *okButton;
	QLineEdit *startTemp, *endTemp;
	QLabel *startTempL, *endTempL, *note;

	int sTemp, eTemp;
	bool useInterval;

	pcl::visualization::PCLVisualizer viewer;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

public Q_SLOTS:

	void setTempInterval()
	{
		bool gotInt;
		useInterval = false;
		int start = startTemp->text().toInt(&gotInt);
		if (!gotInt)
		{
			note->setText("Error: <br />Start temp. is not an int!");
		}
		else
		{
			int end = endTemp->text().toInt(&gotInt);
			if (!gotInt)
			{
				note->setText("Error: <br />End temp. is not an int!");
			}
			else
			{
				if (end < start)
				{
					note->setText("Error: End temp. is smaller <br />than start temp.!");
				}
				else
				{
					sTemp = start;
					eTemp = end;
					useInterval = true;
					note->setText(QString("Set temperature range <br />from ") +
								  QString::number(start) +  QChar(0x00B0) + QString("C to ") +
								  QString::number(end) + QChar(0x00B0) + QString("C!"));
				}
			}
		}
	}

	void updatePointCloud(pcl::PointCloud<pcl::PointXYZRGB> &newCloud, std::vector<float> &newTemps)
	{
		if (newCloud.size() == newTemps.size())
		{
			pcl::copyPointCloud(newCloud, *cloud);
			if (useInterval)
			{
				uint32_t rgb = ((uint32_t)0 << 16 | (uint32_t)255 << 8 | (uint32_t)0);
				for (size_t i = 0; i < cloud->size(); i++)
				{
					if (newTemps[i] >= sTemp && newTemps[i] < eTemp)
						cloud->points[i].rgb = *reinterpret_cast<float*>(&rgb);

				}
			}
			pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
			viewer.updatePointCloud<pcl::PointXYZRGB>(cloud, rgb, "thermo_cloud");
		}

	}

};


