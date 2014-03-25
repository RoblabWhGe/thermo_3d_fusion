#ifndef VIDEODEVICE_H
#define VIDEODEVICE_H

#include <string.h>
#include <stdlib.h>


#define CLEAR(x) memset(&(x), 0, sizeof(x))

class VideoDevice
{
public:
    struct frame_size
    {
        int width;
        int height;
    };

    VideoDevice();
    int open_device(const char *path);
    int close_device();
    int init_device(int width, int height);
    int start_capturing();
    int stop_capturing();
    int uninit_device();
    int get_frame(void **, size_t*);
    int release_frame();
    int get_framesize_type(int &,struct frame_size **);
    int get_framesize(int *frame_width, int *frame_height,int *frequency);

private:
    int init_mmap();
    struct buffer
    {
        void * start;
        size_t length;
    };
    char *dev_name;
    int fd;
    buffer* buffers;
    unsigned int n_buffers;
    int index;
    fd_set fds;
    struct timeval tv;

};

#endif // VIDEODEVICE_H
