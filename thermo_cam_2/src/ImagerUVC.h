#ifndef IMAGERUVC
#define IMAGERUVC

#include "VideoDevice.h"

class ImagerUVC
{
public:

  /**
   * Standard constructor
   */
  ImagerUVC(const char* dev_path=NULL);

  /**
   * Destructor
   */
  ~ImagerUVC();

  /**
   * Find first device with vendor and product identifier of Optris GmbH
   * @return Serial number (if successfully found)
   */
  unsigned long FindFirstDevice();

  /**
   * Find device with specific serial number of Optris GmbH
   * @return Serial number (if successfully found)
   */
  unsigned long FindDeviceBySerial(unsigned long serial);

  /**
   * Get UVC path
   * @return UVC path
   */
  char* GetPath();

  /**
   * Open device
   * @return success
   */
  int OpenDevice();

  /**
   * Start video grabbing
   */
  void Start();

  /**
   * Stop video grabbing
   */
  void Stop();

  /**
   * Close device
   * @return success
   */
  int CloseDevice();

  /**
   * Get image width
   */
  unsigned int GetWidth();

  /**
   * Get image height
   */
  unsigned int GetHeight();

  /**
   * Get frequency of video stream
   */
  unsigned int GetFrequency();

  /**
   * Acquire one frame
   */
  int GetFrame(unsigned char *buffer);

  /**
   * Release frame binding
   */
  int ReleaseFrame();

private:

  /**
   * Generic find method
   * @param[in] serial Serial number to be found, if serial==0, then first device is returned
   * @return Serial number (if successfully found)
   */
  unsigned long FindDevice(unsigned long serial);

  VideoDevice *_dev;

  char* _path;

  int _height;

  int _width;

  int _freq;

  unsigned char *_buf;

  size_t _len;
};

#endif // IMAGERUVC
