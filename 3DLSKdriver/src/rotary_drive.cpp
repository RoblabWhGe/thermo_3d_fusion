/*
 * Initial Version
 * @author Thorsten Linder
 * @date 18.1.2011
 *
 * Modified Version
 * @author Daniel
 * @date 25.09.2012
 *
 * Redesign Version
 * @author Werner Halft (werner.halft@iais.fraunhofer.de)
 * @date 22.10.2012
 *
 */

/* Offene Fragen:
 * High Presicission Encoder: roll = 1 or roll = -1 ... Why?
 * Zero Pose Transformation: magic number? 0.115?
 */

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_broadcaster.h>

#include "fair/core/io/IDeviceAdapter.h"
#include "fair/core/io/com/CComAdapter.h"
#include "fair/devices/servo/CEposVMC.h"
#include "fair/core/base/CTimer.h"

#include <math.h>

const std::string ROTARY_DRIVE = "rotary_drive";
const bool HIGH_PRECISSION_ENCODER = true;

// usb port
const std::string ROTARY_DRIVE_PARAM_NAME_PORT = "/" + ROTARY_DRIVE + "/port";
const std::string ROTARY_DRIVE_DEFAULT_PORT = "/dev/ttyUSB0";
std::string rotaryDrivePort = ROTARY_DRIVE_DEFAULT_PORT;

// rotates per minutes
const std::string ROTARY_DRIVE_PARAM_NAME_RPM = "/" + ROTARY_DRIVE + "/rpm";
const int ROTARY_DRIVE_DEFAULT_RPM = 500;
int rotaryDriveRpm = ROTARY_DRIVE_DEFAULT_RPM;
const int ROTARY_DRIVE_MIN_RPM = 40;
const int ROTARY_DRIVE_MAX_RPM = 1500;

// ticks per turn
const std::string ROTARY_DRIVE_PARAM_NAME_TICKS = "/" + ROTARY_DRIVE + "/ticks";
const int ROTARY_DRIVE_DEFAULT_TICKS = 144;
int rotaryDriveTicks = ROTARY_DRIVE_DEFAULT_TICKS;
const int ROTARY_DRIVE_MEAN_WINDOW_SIZE = 5;

int main(int argc, char** argv) {
    ros::init(argc, argv, ROTARY_DRIVE);
    ros::NodeHandle nodeHandle;

    tf::TransformBroadcaster tf_broadcaster;

    ROS_INFO("%s", ("init node: /" + ROTARY_DRIVE).c_str());

    // port
    nodeHandle.getParam(ROTARY_DRIVE_PARAM_NAME_PORT, rotaryDrivePort);
    ROS_INFO("%s", ("set param " + ROTARY_DRIVE_PARAM_NAME_PORT + " = " + rotaryDrivePort).c_str());
    ros::Duration(0.5).sleep();

    // rpm
    nodeHandle.getParam(ROTARY_DRIVE_PARAM_NAME_RPM, rotaryDriveRpm);
    ROS_INFO("%s %d", ("set param " + ROTARY_DRIVE_PARAM_NAME_RPM + " =").c_str(), rotaryDriveRpm);
    ros::Duration(0.5).sleep();

    // ticks per turn
    nodeHandle.getParam(ROTARY_DRIVE_PARAM_NAME_TICKS, rotaryDriveTicks);
    ROS_INFO("%s %d", ("set param " + ROTARY_DRIVE_PARAM_NAME_TICKS + " =").c_str(), rotaryDriveTicks);
    ros::Duration(0.5).sleep();

    // motor
    ROS_INFO("init motor controller ...");
    char** rotaryDrivePortArray = new char*[2];
    rotaryDrivePortArray[1] = (char*) rotaryDrivePort.c_str();
    fair::CComAdapter* motorControllerAdapter = new fair::CComAdapter(rotaryDrivePortArray[1]);
    motorControllerAdapter->openDevice();

    const unsigned int acceleration = ROTARY_DRIVE_MIN_RPM < rotaryDriveRpm ? rotaryDriveRpm / 2 : ROTARY_DRIVE_MIN_RPM;
    epos::CEposVMC* eposVirtualMotorController = new epos::CEposVMC(2, rotaryDrivePortArray);
    eposVirtualMotorController->profileVelocityMode(rotaryDriveRpm, ROTARY_DRIVE_MAX_RPM, acceleration,
            acceleration * 2);

    // zero pose
    ROS_INFO("init motor zero pose ...");
    long startingOffset;
    tf::Transform transform;
    if ( motorControllerAdapter->getRingIndicator() ) {
        eposVirtualMotorController->getActualPosition(&startingOffset);
        transform.setOrigin(tf::Vector3(0.0, 0.0, 0.115));
        transform.setRotation(tf::createQuaternionFromRPY(0.0, 0.0, 0));
        tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "rotary_base", "rotary_plate"));
    }

    // determine ticks per turn
    long current_ticks = 0;
    if ( rotaryDriveTicks == -1 ) {
        ROS_INFO("determine motor ticks-per-turn ...");
        for ( int turn = 0; turn < ROTARY_DRIVE_MEAN_WINDOW_SIZE; turn++ ) {
            ROS_INFO("   turn %d of %d", turn+1, ROTARY_DRIVE_MEAN_WINDOW_SIZE);
            while ( !motorControllerAdapter->getRingIndicator() ) {
                // just wait for ring indicator position
            }
            eposVirtualMotorController->getActualPosition(&current_ticks);
        }
        rotaryDriveTicks = (current_ticks - startingOffset) / ROTARY_DRIVE_MEAN_WINDOW_SIZE;
        startingOffset = current_ticks;
        ROS_INFO("   set param tick-per-turn = %d", rotaryDriveTicks);
    }

    // everything is fine
    ROS_INFO("%s", ("node /" + ROTARY_DRIVE + " is runnig").c_str());

    // publishing topic
    double delta_ticks = 0;
    long sum = 0;
    ros::Publisher publishTwistTopic = nodeHandle.advertise<geometry_msgs::Twist>("/" + ROTARY_DRIVE + "/twist", 10);
    while ( nodeHandle.ok() ) {
        if ( 0 == eposVirtualMotorController->getActualPosition(&current_ticks) ) {
            delta_ticks = current_ticks - startingOffset;
            sum += delta_ticks;

            double roll = 1 * ((sum % rotaryDriveTicks) / (double) rotaryDriveTicks) * 2.0 * M_PI;
            if ( HIGH_PRECISSION_ENCODER ) {
                roll = -1 * roll;
            }
            transform.setRotation(tf::createQuaternionFromRPY(0, 0, roll));
            startingOffset = current_ticks;
            geometry_msgs::Twist twist;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = roll;
            publishTwistTopic.publish(twist);
            tf_broadcaster.sendTransform(
                    tf::StampedTransform(transform, ros::Time::now(), "rotary_base", "rotary_plate"));
            ros::Duration(0.1).sleep();
            ros::spinOnce();
        }
    }

    // shut down
    eposVirtualMotorController->CloseDevice();
    ROS_INFO("%s", ("node /" + ROTARY_DRIVE + " finished").c_str());

    return 0;
}
