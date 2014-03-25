#ifndef IMAGER_H
#define IMAGER_H

namespace optris
{

typedef void (*fptrOptrisFrame)(unsigned short* data, unsigned int w, unsigned int h);

/**
 * @class Imager
 * @brief Wrapper for optris driver and image processing library
 * @author Stefan May
 */
class Imager
{
public:

  /**
   * Standard constructor
   * @param[in] serial Serial number
   * @param[in] width Image width
   * @param[in] height Image height
   * @param[in] freq Frequency
   * @param[in] path Path to XML-configuration file
   */
  Imager(unsigned long serial, unsigned int width, unsigned int height, unsigned int freq, char* path);

  /**
   * Standard constructor
   * @param[in] serial Serial number
   * @param[in] width Image width
   * @param[in] height Image height
   * @param[in] freq Frequency
   * @param[in] fov Field of view
   * @param[in] minTemp Minimum temperature
   * @param[in] maxTemp Maximum temperature
   * @param[in] framerate Interpolated framerate of camera
   */
  Imager(unsigned long serial, unsigned int width, unsigned int height, unsigned int freq, int fov, float minTemp, float maxTemp, float framerate = 30.0);

  /**
   * Destructor
   */
  ~Imager();

  /**
   * Get image width
   * @return Image width, i.e. number of columns
   */
  unsigned int getWidth();

  /**
   * Get image height
   * @return Image height, i.e. number of rows
   */
  unsigned int getHeight();

  /**
   * Get image size
   * @return Number of pixels, i.e. rows x columns
   */
  unsigned int getSize();

  /**
   * Set callback function to be called for new frames
   * @param[in] callback Pointer to callback function
   */
  void setFrameCallback(fptrOptrisFrame callback);

  /**
   * Enable/Disable autoflag control feature
   * @param[in] control Automatic flag control
   */
  void setAutoflagControl(bool control);

  /**
   * Force Recalibration within next image processing cycles
   */
  void forceRecalibration();

  /**
   * Process imager data
   * @param[in] buffer Raw data acquired from device
   */
  void process(unsigned char* buffer);

private:

  void init(unsigned long serial, unsigned int width, unsigned int height, unsigned int freq, int fov, float minTemp, float maxTemp, float framerate);

  /**
   * Last temperature measurement with box sensor
   */
  float _tempBoxLast;

  /**
   * Last temperature measurement with flag sensor
   */
  float _tempFlagLast;

  /**
   * Last temperature measurement with chip sensor
   */
  float _tempChipLast;

  /**
   * Automatic flag control feature
   */
  bool _autoFlagControl;

  /**
   * Force recalibration
   */
  bool _forceRecalib;
};

}

#endif
