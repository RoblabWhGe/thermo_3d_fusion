#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <thermo_cam_2/ThermoData.h>
#include <std_srvs/Empty.h>
#include "Imager.h"
#include "ImagerUVC.h"
#include "IronPalette.h"
#include <camera_info_manager/camera_info_manager.h>

/** Global publisher for the thermo images */
ros::Publisher imagePublisher;

/** Global publisher for the info messages */
ros::Publisher infoPublisher;

/** Global publisher for the thermo data */
ros::Publisher dataPublisher;

/** Global flag wheather the camera should recalibrate before proccessing next frame */
bool recalibrateCamera;

/** Global min. and max. temperature values */
int gMinTemp, gMaxTemp;

/** Global cameraInfoManager */
camera_info_manager::CameraInfoManager *cinfo;

bool calibrated = true;

/**
 * Converts the thermo camera buffer into a colored thermo image.
 * The image is drawn using the iron color palette with dynmic temperature
 * ranges.
 *
 * @param image Buffer from the thermo camera
 * @param w The width of the camera image
 * @param h The height of the camera image
 */
void publishColorImage(unsigned short *image, unsigned int w, unsigned int h, ros::Time &tStamp)
{
	/* Get current CameraInfo data */
	sensor_msgs::CameraInfoPtr ci(new sensor_msgs::CameraInfo(cinfo->getCameraInfo()));

	/* Get Min/Max Temperature for dynamic scaling */
	unsigned short minTemp = USHRT_MAX;
	unsigned short maxTemp = 0;

	if (gMinTemp == -280)
	{
		for (int i = 0; i < (w * h); i++)
		{
			unsigned short value = image[i];

			if (value > maxTemp)
			{
				maxTemp = value;
			}
			if (value < minTemp)
			{
				minTemp = value;
			}
		}
	}
	else
	{
		minTemp = gMinTemp;
		maxTemp = gMaxTemp;
	}



    sensor_msgs::Image currentFrame;
    currentFrame.header.frame_id = "thermo_cam";
    currentFrame.header.stamp = tStamp;
    currentFrame.encoding = "rgb8";
    currentFrame.height = h;
    currentFrame.width = w;

    /* Convert buffer to image */
    for (int i = 0; i < (w * h); i++)
    {
    	if (image[i] < minTemp)
    	{
    		currentFrame.data.push_back(0);
			currentFrame.data.push_back(0);
			currentFrame.data.push_back(0);
    	}
    	else if (image[i] > maxTemp)
    	{
    		currentFrame.data.push_back(255);
			currentFrame.data.push_back(255);
			currentFrame.data.push_back(255);
    	}
    	else
    	{
			/* Get converted color for temperature from the iron palette.
			   Compare to minTemp to avoid zero divide exception */
			int ironIdx = (image[i] == minTemp)
							? 0
							: (image[i] - minTemp) * 239 / (maxTemp - minTemp);
			currentFrame.data.push_back(Iron[ironIdx][0]);
			currentFrame.data.push_back(Iron[ironIdx][1]);
			currentFrame.data.push_back(Iron[ironIdx][2]);
    	}
    }

    if (ci->K[0] != 0.0 && (ci->height != h || ci->width != w))
    {
    	if (calibrated)
    	{
    		ROS_WARN_STREAM("[thermo_cam] calibration does not match video mode "
    		    			<< "(publishing uncalibrated data)");
			calibrated = false;
    	}
    	ci.reset(new sensor_msgs::CameraInfo());

    }

    if (ci->K[0] != 0.0)
    {
    	ci->height = h;
		ci->width = w;
    }

    ci->header.frame_id = currentFrame.header.frame_id;
    ci->header.stamp = currentFrame.header.stamp;

    imagePublisher.publish(currentFrame);
    infoPublisher.publish(ci);
}

/**
 * Converts the thermo camera buffer into a data message which contains
 * the temperature values in °C.
 *
 * @param image Buffer from the thermo camera
 * @param w The width of the camera image
 * @param h The height of the camera image
 */
void publishTemperatureData(unsigned short *image, unsigned int w, unsigned int h, ros::Time &tStamp)
{
    thermo_cam_2::ThermoData thermoData;
    thermoData.header.frame_id = "thermo_cam";
    thermoData.header.stamp = tStamp;
    thermoData.width = w;
    thermoData.height = h;

    for (int i = 0; i < (w * h); i++)
    {
        double value = (image[i] - 1000) / 10;
        thermoData.data.push_back(value);
    }
    dataPublisher.publish(thermoData);
}

/**
 * Callback function for the thermo camera image buffer.
 *
 * @param image Buffer from the thermo camera
 * @param w The width of the camera image
 * @param h The height of the camera image
 */
void cbOnFrame(unsigned short *image, unsigned int w, unsigned int h)
{
	ros::Time tStamp = ros::Time::now();
    publishColorImage(image, w, h, tStamp);
    publishTemperatureData(image, w, h, tStamp);
}

/**
 * Callback funtion for the camera calibration service.
 * The function simply sets a global flag, so the camera can recalibrate
 * before processing the next frame.
 *
 * @param req Not used
 * @param res Not used
 * @return true if service was handled successfull
 */
bool cameraRecalibrationServiceCallback(std_srvs::EmptyRequest &req,
                                        std_srvs::EmptyResponse &res)
{
    recalibrateCamera = true;

    return true;
}

/**
 * Main Function:
 * Initializes the ros environment, starts the uvc driver and process
 * the thermo images via the thermo imager library.
 *
 * @param argc Number of given arguments
 * @param argv List of given arguments
 * @return EXIT_SUCCESS if node finishes successfully
 */
int main (int argc, char* argv[])
{
    ros::init(argc, argv, "thermo_cam_2");
    ros::NodeHandle nh("thermo_cam");
    ros::NodeHandle privNh ("~");
    ros::ServiceServer cameraRecalibrationService;
    cinfo = new camera_info_manager::CameraInfoManager(nh,"thermo_cam");

    /* Get CameraInfo */
    std::string info_url;
    privNh.param<std::string>("info_url", info_url, "package://thermo_cam_2/cali/cali.yaml");
    cinfo->loadCameraInfo(info_url);

    /* Get config path filename */
    std::string config_file;
    privNh.param<std::string>("config_file", config_file, "10110034.xml");
    config_file = ros::package::getPath("thermo_cam_2")
            + "/config/" + config_file;

    /* Get min. and max. temperature values */
    privNh.param<int>("min_temp", gMinTemp, -280);
    privNh.param<int>("max_temp", gMaxTemp, -280);
    gMaxTemp = (gMinTemp == -280)? -280: gMaxTemp * 10 + 1000;
    gMinTemp = (gMaxTemp == -280)? -280: gMinTemp * 10 + 1000;

    /* Anounce publisher */
    imagePublisher = nh.advertise<sensor_msgs::Image>("/thermo_cam/image_raw", 1);
    infoPublisher = nh.advertise<sensor_msgs::CameraInfo>("/thermo_cam/camera_info", 1);
    dataPublisher = nh.advertise<thermo_cam_2::ThermoData>("/thermo_cam/thermo_data", 1);

    /* Anounce service for camera recalibration */
    cameraRecalibrationService = nh.advertiseService("/thermo_cam/recalibrate_thermo_cam",
                                            cameraRecalibrationServiceCallback);
    recalibrateCamera = false;

    /* Initialize UVC interface and determine serial number of Optris imager */
	ImagerUVC uvc;
	unsigned long serial = uvc.FindFirstDevice();
    if(serial == 0)
	{
        ROS_ERROR("Error finding imager device");
        return EXIT_FAILURE;
	}

	if(uvc.OpenDevice()!=0)
	{
        ROS_ERROR("Error opening UVC device: %s", uvc.GetPath());
        return EXIT_FAILURE;
	}

    /* Initialize Optris image processing chain */
    optris::Imager im(serial, uvc.GetWidth(), uvc.GetHeight(),
                      uvc.GetFrequency(), (char *)config_file.c_str());
	im.setAutoflagControl(true);
    im.setFrameCallback(cbOnFrame);
	uvc.Start();

    /* Create buffer for image processing */
    unsigned char bufferRaw[uvc.GetWidth() * uvc.GetHeight() * 2];

    /* Endless loop in order to pass raw data to image processing library */
    while(ros::ok())
	{
        if (recalibrateCamera)
        {
            im.forceRecalibration();
            recalibrateCamera = false;
        }

        /* Process buffer */
		uvc.GetFrame(bufferRaw);
		im.process(bufferRaw);
		uvc.ReleaseFrame();

        ros::spinOnce();
	}
	uvc.Stop();
    uvc.CloseDevice();

    return EXIT_SUCCESS;
}
