///*
// * scan2cloud.cpp
// *
// *  Created on: Mar 28, 2011
// *      Author: tlinder
// */
//
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <iostream>
#include <fstream>

/**
 * Keeps the last read scan
 */
sensor_msgs::PointCloud2 _cloud_in;

ros::Subscriber _cloud_sub;

/**
 * Counts the number of stored PCD files
 */unsigned int pcdCount;

/**
 * Keeps the name of the scan frame. Set by ROS parameter
 */
//	std::string scan_frame1;
/**
 * Keeps the name of the target/fixed frame. Set by ROS parameter
 */
std::string target_frame;
/**
 *
 */
std::string pcdFileName, filename_csv_output;

double cloudTime;
int cloudCounter;
std::ofstream fout;

void savePCD(std::string _filename, sensor_msgs::PointCloud2 *_cloud) {
    ROS_INFO("Storing PCD-File (\"%s\") width %u points", _filename.c_str(), _cloud->width);
    if ( -1 == pcl::io::savePCDFile(_filename, *_cloud) ) {
        ROS_ERROR("PCD-File \"%s\" could not been stored!", _filename.c_str());
    }
}

/***
 *\brief Callback function that collects all scans and converts to pointcloud2
 *\param  LaserScan
 *
 */
void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_ptr) {
    cloudCounter++;
    if ( (cloudCounter % 4) == 0 ) {
        sensor_msgs::PointCloud2 sub_cloud;
        _cloud_in = *cloud_ptr;
        std::stringstream _filenamePlusNo;
        if ( 0 != pcdFileName.size() ) {

            _filenamePlusNo << pcdFileName << pcdCount << ".pcd";
            savePCD(_filenamePlusNo.str(), &_cloud_in);
            pcdCount++;
        }ROS_INFO("Time Delta %f", _cloud_in.header.stamp.toSec() - cloudTime);
        fout << _filenamePlusNo.str() << ";" << _cloud_in.header.stamp.toSec() << ";"
                << _cloud_in.header.stamp.toSec() - cloudTime << "\n";
        fout.flush();
        cloudTime = _cloud_in.header.stamp.toSec();
    }
}

int main(int argc, char** argv) {

    ros::init(argc, argv, "cloud2PCD");
    ros::NodeHandle nodeHandle;
    pcdFileName = "3dlsk_";
    filename_csv_output = pcdFileName + "time.csv";
    pcdCount = 0;
    _cloud_sub = nodeHandle.subscribe("/cloud", 10, cloudCallback);
//PointCloud2ToPCD *scan2PointCloud2 = new PointCloud2ToPCD(nodeHandle);
    cloudTime = 0;
    cloudCounter = 0;
    fout.open(filename_csv_output.c_str(), std::ofstream::app);
    fout.precision(18);
    if ( !fout ) {
        ROS_ERROR("Couldn't open csv file.");
        return -1;
    } else {
        fout << "File Name;Moment of Cloud Capturing; Delta between Clouds\n";
    }

    ROS_INFO("Node started");
    while ( nodeHandle.ok() ) {

        ros::spinOnce();

    }
    fout.close();
    ROS_INFO("Node finished");
    nodeHandle.shutdown();
    return 0;
}
