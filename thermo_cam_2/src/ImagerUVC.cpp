#include "ImagerUVC.h"
#include <iostream>
#include <libudev.h>

using namespace std;

ImagerUVC::ImagerUVC(const char* dev_path)
{
  _path = NULL;

  if(dev_path!=NULL)
  {
    _path = new char[strlen(dev_path)+1];
    memcpy(_path, dev_path, strlen(dev_path));
    _path[strlen(dev_path)] = '\0';
  }
  _dev  = new VideoDevice();
}

ImagerUVC::~ImagerUVC()
{
  if(_path!=NULL)
  {
    delete [] _path;
    _path = NULL;
  }
  delete _dev;
}

unsigned long ImagerUVC::FindFirstDevice()
{
  return FindDevice(0);
}

unsigned long ImagerUVC::FindDeviceBySerial(unsigned long serial)
{
  return FindDevice(serial);
}

unsigned long ImagerUVC::FindDevice(unsigned long serial)
{
  bool findFirst = (serial==0);

  unsigned long retval = 0;

  if(_path!=NULL)
  {
    delete [] _path;
    _path = NULL;
  }

  struct udev *udev;
  struct udev_enumerate *enumerate;
  struct udev_list_entry *devices, *dev_list_entry;

  udev      = udev_new();
  enumerate = udev_enumerate_new(udev);
  udev_enumerate_add_match_subsystem(enumerate, "video4linux");
  udev_enumerate_scan_devices(enumerate);
  devices   = udev_enumerate_get_list_entry(enumerate);

  udev_list_entry_foreach(dev_list_entry, devices)
  {
    const char *sysfs_path;
    const char *dev_path;
    const char *str;
    struct udev_device *uvc_dev; // The device's UVC udev node.
    struct udev_device *dev; // The actual hardware device.

    sysfs_path  = udev_list_entry_get_name(dev_list_entry);
    uvc_dev     = udev_device_new_from_syspath(udev, sysfs_path);
    dev_path    = udev_device_get_devnode(uvc_dev);

    dev = udev_device_get_parent_with_subsystem_devtype( uvc_dev, "usb", "usb_device");
    const char * strVendor = udev_device_get_sysattr_value(dev, "idVendor");
    const char* strProduct = udev_device_get_sysattr_value(dev, "idProduct");

    str = udev_device_get_sysattr_value(dev, "serial");
    unsigned long serno   = (str)? strtol(str, NULL, 10): 0x0;
    char vendorOptris[]   = "0403";
    char productOptris[]  = "de37";
    if(strcmp(strVendor, vendorOptris)==0 && strcmp(strProduct, productOptris)==0 && (serno==serial || findFirst))
    {
      _path = new char[strlen(dev_path)+1];
      memcpy(_path, dev_path, strlen(dev_path));
      _path[strlen(dev_path)] = '\0';
      retval = serno;
    }

    udev_device_unref(uvc_dev);

    if(retval != 0) break;
  }

  udev_enumerate_unref(enumerate);
  udev_unref(udev);

  return retval;
}

char* ImagerUVC::GetPath()
{
  return _path;
}

int ImagerUVC::OpenDevice()
{
  int ret = _dev->open_device(_path);
  if(ret!=0)
  {
    cout << "ImagerUVC::OpenDevice: Error opening video device " << _path << endl;
    return -1;
  }
  ret = _dev->get_framesize(&_width, &_height, &_freq);
  if(ret!=0)
  {
    cout << "ImagerUVC::OpenDevice: Error in query for image format " << endl;
    return -1;
  }
  ret = _dev->init_device(_width, _height);
  if(ret!=0)
  {
    cout << "ImagerUVC::InitDevice: Error initializing video device" << endl;
    return -1;
  }
  return ret;
}

void ImagerUVC::Start()
{
  _dev->start_capturing();
}

void ImagerUVC::Stop()
{
  _dev->stop_capturing();
}
int ImagerUVC::CloseDevice()
{
  return _dev->close_device();
}

unsigned int ImagerUVC::GetWidth()
{
  return _width;
}

unsigned int ImagerUVC::GetHeight()
{
  return _height;
}

unsigned int ImagerUVC::GetFrequency()
{
  return _freq;
}

int ImagerUVC::GetFrame(unsigned char *buffer)
{
  int in,out = 0;
  int ret = _dev->get_frame((void **)&_buf, &_len);
  if(ret<0)
  {
    Stop();
    CloseDevice();
    if(OpenDevice()<0)
    {
      exit(1);
    };
    Start();
    return GetFrame(buffer);
  };
  if(ret == 0)
  {
    for(in = 0; in < _width * _height * 2; in++){
      buffer[out++] = _buf[in];
    }
  }
  return (ret==0);
}

int ImagerUVC::ReleaseFrame()
{
  int ret;
  ret = _dev->release_frame();
  return ret;
}
