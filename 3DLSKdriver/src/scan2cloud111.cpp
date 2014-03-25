#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl/io/pcd_io.h>
//#include <pcl/io/vtk_io.h>
//#include <vtk-5.2/vtkX3DExporter.h>
#include <pcl/point_types.h>
//#include "pcl_visualization/cloud_viewer.h"
#include "pcl/point_cloud.h"
//#include <pcl/sample_consensus/model_types.h>
//#include <pcl/sample_consensus/method_types.h>
//#include <pcl/segmentation/sac_segmentation.h>
#include "pcl/ros/conversions.h"
#include <sys/types.h>
#include <unistd.h>

using namespace std;

class ScanToPointCloud2For2Lasers {
private:

	double _the_last_1st_yaw;
	double _the_last_2nd_yaw;
	sensor_msgs::PointCloud2 _resulting_cloud;

	double _start_yaw;

	unsigned int producedPcdCount; // how many pcd ALREADY produced
	unsigned int maxToProducePcdCount; // how many pcd can be maximal to produce
	bool isSkipThisCloud; // it indicates if the current cloud is incomplete and should be skipped

	bool bRunning;

	// ----------------------

	std::string target_frame;
	ros::NodeHandle _nodeHandle;
	laser_geometry::LaserProjection _laserProjector;
	tf::TransformListener _tfListener;
	message_filters::Subscriber<sensor_msgs::LaserScan> _laser_sub;
	tf::MessageFilter<sensor_msgs::LaserScan> _laser_tfMessagefilter;
	ros::Publisher _scan_pub;

	string pcd_prefix;

public:

	bool isRunning() {
		return bRunning;
	}

	void setPcdPrefix(string prefix) {
		pcd_prefix = prefix;
	}

	ScanToPointCloud2For2Lasers(ros::NodeHandle n) :
		_nodeHandle(n),
		target_frame("/rotary_base"),
		_laser_sub(_nodeHandle, "/scan",300),
		_laser_tfMessagefilter(_laser_sub, _tfListener, "/laser", 300),
		producedPcdCount(0),
		maxToProducePcdCount(5),
		bRunning(true),
		isSkipThisCloud(true), // set it to true to skip the first cloud
		_the_last_1st_yaw(0),
		_the_last_2nd_yaw(0)
	{
		_laser_tfMessagefilter.setTolerance(ros::Duration(0.1));
		_laser_tfMessagefilter.registerCallback(boost::bind(&ScanToPointCloud2For2Lasers::scanCallback, this, _1));

		_scan_pub = _nodeHandle.advertise<sensor_msgs::PointCloud2> ("/cloud", 100);

		std::stringstream pid;
		pid << getpid();
		pcd_prefix = pid.str();

	}

	~ScanToPointCloud2For2Lasers() {
		delete (&_resulting_cloud);
		delete (&_laserProjector);
		delete (&_tfListener);
		delete (&_laser_tfMessagefilter);
	}

	void getRPY(tf::Quaternion quad, float &roll, float &pitch, float &yaw) {
		tfScalar btroll,btpitch,btyaw;
		tf::Matrix3x3(quad).getRPY(btroll, btpitch, btyaw);

		roll = btroll;
		pitch = btyaw; // vertauscht!!
		yaw = btpitch; // vertauscht!!
//		roll = atan2(2 * (quad.getX() * quad.getY() + quad.getW() * quad.getZ()), quad.getW() * quad.getW()
//				+ quad.getX() * quad.getX() - quad.getY() * quad.getY() - quad.getZ() * quad.getZ());
//
//		pitch = atan2(2 * (quad.getY() * quad.getZ() + quad.getW() * quad.getX()),
//				quad.getW() * quad.getW() - quad.getX() * quad.getX() - quad.getY() * quad.getY() + quad.getZ() * quad.getZ());
//
//		yaw = asin(-2 * (quad.getX() * quad.getZ() - quad.getW() * quad.getY()));

//		cout << roll <<" " << pitch << " " <<yaw <<  endl;
//		cout << btroll <<" " << btpitch << " " <<btyaw <<  endl;
//		cout << "....." << endl;
	}

	void resetCloudCollectionOnError() {
		_the_last_1st_yaw = _the_last_2nd_yaw = 0;
		isSkipThisCloud = true; // it may be set to false to ignore any error during the scan
	}

	/***
	 *\brief Callback function that collects all scans and converts to pointcloud2
	 *\param  LaserScan
	 *
	 */
	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_ptr) {

		sensor_msgs::LaserScan _scan_in = *scan_ptr;

		if (_scan_in.header.frame_id.compare("/laser") != 0) {
			ROS_WARN("An unknown source: %s", _scan_in.header.frame_id);
			return;
		}

		if (maxToProducePcdCount > 0 && producedPcdCount >= maxToProducePcdCount) {
			//ROS_WARN("Already %d point cloud produced", maxToProducePcdCount);
			bRunning = false;
			return;
		}

		ros::Time originalTime = _scan_in.header.stamp;
		_scan_in.header.stamp = originalTime - ros::Duration(0.05); // hack: Verzoegerung

		if (!_tfListener.waitForTransform( /* wait for possible TF */
				target_frame,
				_scan_in.header.frame_id,
				_scan_in.header.stamp, ros::Duration(0.01)))
		{
			resetCloudCollectionOnError();
			ROS_WARN("Timeout with waitForTransform");
			return;
		}

		sensor_msgs::PointCloud2 sub_cloud;
		tf::StampedTransform transform;

		try { /* transform laser scan to a sub_cloud */

			_laserProjector.transformLaserScanToPointCloud(
					target_frame,
					_scan_in,
					sub_cloud,
					_tfListener,
					-1.0, laser_geometry::channel_option::Default);

		} catch (tf::TransformException& exc) {
			resetCloudCollectionOnError();
			ROS_ERROR("Failed with transformLaserScanToPointCloud: %s", exc.what());
			return;
		}

		try {
			//Get the searched TF.  Throws exception on failure.
			_tfListener.lookupTransform(_scan_in.header.frame_id,
					target_frame, _scan_in.header.stamp, transform);
		} catch (tf::TransformException &ex) {
			resetCloudCollectionOnError();
			ROS_ERROR("Failed with lookupTransform %s\n", ex.what()); //Print exception which was caught
			return;
		}

		if (!isSkipThisCloud) {
			sensor_msgs::PointCloud2 tmp_cloud = _resulting_cloud;
			pcl::concatenatePointCloud(tmp_cloud, sub_cloud, _resulting_cloud);
		}

		float roll,pitch,yaw;
		getRPY(transform.getRotation(),roll,pitch,yaw);
		const double current_yaw = yaw;

		bool isCrossZeroIndicator = false;
		if ((0 < _the_last_2nd_yaw
				&& 0 < _the_last_1st_yaw
				&& 0 > current_yaw)  ||
			(0 > _the_last_2nd_yaw
				&& 0 > _the_last_1st_yaw
				&& 0 < current_yaw)) {

			isCrossZeroIndicator = true;

		}

		if (isCrossZeroIndicator) {
			// we have now a possible complete scan !
			if(!isSkipThisCloud)
			{
				producedPcdCount++;
				ROS_INFO("Collected %d cloud with size [%d,%d]", producedPcdCount, _resulting_cloud.width, _resulting_cloud.height);

				_resulting_cloud.header.stamp = _scan_in.header.stamp;
				_scan_pub.publish(_resulting_cloud);

				pcl::PointCloud<pcl::PointXYZI> cloud;
				pcl::fromROSMsg(_resulting_cloud, cloud);

				time_t rawtime;
				time(&rawtime);
				std::stringstream newtimestamp;
				newtimestamp << pcd_prefix << "_" << rawtime;
				//pcd/" + newtimestamp.str() + "
				std::string strStamp = "pcd/" + newtimestamp.str() + "_lms111.pcd";

				pcl::io::savePCDFileASCII (strStamp, cloud);
			} else {
				ROS_WARN("Skipped incomplete cloud due to some error data");
			}

			isSkipThisCloud = false;
			_start_yaw = current_yaw;
			_resulting_cloud = sub_cloud; // reset
		}

		_the_last_2nd_yaw = _the_last_1st_yaw;
		_the_last_1st_yaw = current_yaw;

	}



};

int main(int argc, char** argv) {

	if (argc==1) {
		ROS_INFO("Usage: %s [pcd prefix]", argv[0]);
	}

	ros::init(argc, argv,"scan2pointcloud2For2Lasers");
	ros::NodeHandle nodeHandle;
	ScanToPointCloud2For2Lasers *scan2PointCloud2 = new ScanToPointCloud2For2Lasers(nodeHandle);
	if (argc==2) { // if pcd prefix is defined
		scan2PointCloud2->setPcdPrefix(string(argv[1]));
	}

	ROS_INFO("Node started");
	while (ros::ok()) { // ros::spin();
		ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0.1));
		if (!scan2PointCloud2->isRunning()) {
			break; // break loop;
		}
	}
	ROS_INFO("Node finished");
	nodeHandle.shutdown();
	return 0;
}
