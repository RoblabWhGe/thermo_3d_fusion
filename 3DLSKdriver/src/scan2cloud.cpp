///*
// * scan2cloud.cpp
// *
// *  Created on: Mar 28, 2011
// *      Author: tlinder
// */
//
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include "laser_geometry_bugfix.cpp"

class ScanToPointCloud2 {
private:
	/**
	 * Keeps the last read scan
	 */
	sensor_msgs::LaserScan _scan_in;
	/**
	 * Keeps the last read rotary position
	 */
	double _formerRotationPosition;
	/**
	 * Contains the accumulated cloud (result) which will be published
	 */
	sensor_msgs::PointCloud2 _resulting_cloud;
	/**
	 * Helper variable to ensure different behavior for first scan reading
	 */
	bool _isFirstTimeCall;
	/**
	 * Keeps a reference to the ros node handle
	 */
	ros::NodeHandle _nodeHandle;

	laser_geometry::LaserProjection _laserProjector;
	tf::TransformListener _tfListener;
	message_filters::Subscriber<sensor_msgs::LaserScan> _laser_sub;
	tf::MessageFilter<sensor_msgs::LaserScan> _laser_tfMessagefilter;
	ros::Publisher _scan_pub;

	/**
	 * Counts the number of stored PCD files
	 */
	unsigned int pcdCount;

	int scancount;

public:
	/**
	 * Keeps the name of the scan frame. Set by ROS parameter
	 */
	std::string scan_frame;
	/**
	 * Keeps the name of the target/fixed frame. Set by ROS parameter
	 */
	std::string target_frame;
	/**
	 *
	 */
//	std::string pcdFileName;
	ScanToPointCloud2(ros::NodeHandle n) :
			_nodeHandle(n), scan_frame("/laser"), target_frame("/rotary_base"), _laser_sub(
					_nodeHandle, "/scan", 100),_tfListener(ros::Duration(60)), _laser_tfMessagefilter(
					_laser_sub, _tfListener, "/laser", 100), _isFirstTimeCall(
					true),
			/*pcdFileName("Hokuyo_smoke_"),*/pcdCount(0) {
		_laser_tfMessagefilter.setTolerance(ros::Duration(0.5));
		_laser_tfMessagefilter.registerCallback(
				boost::bind(&ScanToPointCloud2::scanCallback, this, _1));
		_scan_pub = _nodeHandle.advertise<sensor_msgs::PointCloud2>("/cloud",
				10);
		_formerRotationPosition = 0;
	}

	~ScanToPointCloud2() {
		delete (&_scan_in);
		delete (&_resulting_cloud);
		delete (&_laserProjector);
		delete (&_tfListener);
		delete (&_laser_tfMessagefilter);
	}

	float getPitch(tf::Quaternion quad) {
		return atan2(
				2 * (quad.getY() * quad.getZ() + quad.getW() * quad.getX()),
				quad.getW() * quad.getW() - quad.getX() * quad.getX()
						- quad.getY() * quad.getY() + quad.getZ() * quad.getZ());
	}

	float getYaw(tf::Quaternion quad) {
		return asin(
				-2 * (quad.getX() * quad.getZ() - quad.getW() * quad.getY()));
	}

	float getRoll(tf::Quaternion quad) {
		return atan2(
				2 * (quad.getX() * quad.getY() + quad.getW() * quad.getZ()),
				quad.getW() * quad.getW() + quad.getX() * quad.getX()
						- quad.getY() * quad.getY() - quad.getZ() * quad.getZ());
	}

	/***
	 *\brief Callback function that collects all scans and converts to pointcloud2
	 *\param  LaserScan
	 *
	 */
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_ptr) {
		sensor_msgs::PointCloud2 sub_cloud;
		tf::StampedTransform transform;
		_scan_in = *scan_ptr;

		scancount++;

		//Wait for a TF which fits criteria for a maximum duration.  Does not throw any exception, only ROS_WARNING
		if (_tfListener.waitForTransform(target_frame, scan_frame,
				_scan_in.header.stamp, ros::Duration(0.2))) {
			//		ROS_INFO("TF available");
			try {
				transformLaserScanToPointCloud_bugfix(target_frame, _scan_in,
						sub_cloud, _tfListener, -1.0,
						laser_geometry::channel_option::Default);
			} catch (tf::TransformException& exc) {
				ROS_WARN("No transform found.");
				ROS_WARN("%s", exc.what());
				return;
			}

			// Do something with cloud.
			if (_isFirstTimeCall) {
				_resulting_cloud = sub_cloud;
				_isFirstTimeCall = false;
			}
			try {
				//Get the searched TF.  Throws exception on failure.
				_tfListener.lookupTransform(scan_frame, target_frame,
						_scan_in.header.stamp, transform);

				//		    std::cout << "\t pitch  " << getPitch(transform.getRotation()) << "\t roll " << getRoll(
				//			    transform.getRotation()) << "\t yaw " << getYaw(transform.getRotation()) << std::endl;

				//Search for the zero degree position. If found then the turn is completed.
				if ((0 > _formerRotationPosition
						&& 0 <= getYaw(transform.getRotation()))
						|| (0 < _formerRotationPosition
								&& 0 >= getYaw(transform.getRotation()))) {
					ROS_INFO(
							"Creating Cloud with size %u and frame_id %s", (unsigned int)_resulting_cloud.width, _resulting_cloud.header.frame_id.c_str());
					_resulting_cloud.header.stamp = _scan_in.header.stamp;
					_scan_pub.publish(_resulting_cloud);
					if (_resulting_cloud.width / scancount % 541 != 0)
						ROS_INFO_STREAM(
								"Scans: "<< scancount <<"  "<< _resulting_cloud.width / 541 << "  "<< _resulting_cloud.width / scancount);
					scancount = 0;

//			if (0 != pcdFileName.size()) {
//			    std::stringstream _filenamePlusNo;
//			    _filenamePlusNo << pcdFileName << pcdCount << ".pcd";
//			    SavePCDThread _saveThread(_filenamePlusNo.str(), _resulting_cloud);
//			    pcdCount++;
//			}

					_resulting_cloud = sensor_msgs::PointCloud2();
					_resulting_cloud = sub_cloud;
					_formerRotationPosition = getYaw(transform.getRotation());

				} else {
					sensor_msgs::PointCloud2 tmp_cloud = _resulting_cloud;
					pcl::concatenatePointCloud(tmp_cloud, sub_cloud,
							_resulting_cloud);
					_formerRotationPosition = getYaw(transform.getRotation());
				}
			} catch (tf::TransformException &ex) {
				ROS_WARN(" %s\n", ex.what());
				//Print exception which was caught
			}
		}
	}
};

int main(int argc, char** argv) {

	ros::init(argc, argv, "scan2pointcloud2");
	ros::NodeHandle nodeHandle;
	ScanToPointCloud2 *scan2PointCloud2 = new ScanToPointCloud2(nodeHandle);
	ROS_INFO("Node started");
	ros::spin();
	ROS_INFO("Node finished");
	nodeHandle.shutdown();
	return 0;
}
