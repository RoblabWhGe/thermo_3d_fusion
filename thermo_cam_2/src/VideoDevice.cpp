#include "VideoDevice.h"

#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#include <asm/types.h>
#include <linux/videodev2.h>

#include <errno.h>
#include <fcntl.h>

#include <stdio.h>

VideoDevice::VideoDevice()
{
//    this->dev_name = dev_name;
    this->fd = -1;
    this->buffers = NULL;
    this->n_buffers = 0;
    this->index = -1;

}

int VideoDevice::open_device(const char *path)
{
    fd = open(path, O_RDWR);
   // fd = open(dev_name.toStdString().c_str(), O_RDWR|O_NONBLOCK, 0);

    if(-1 == fd)
    {
        return -1;
    }
    return 0;
}

int VideoDevice::close_device()
{
    if(fd>0){
        uninit_device();
    };
    if(-1 == close(fd))
    {
        fd = -1;
        return -1;
    }
    fd = -1;
    return 0;
}

int VideoDevice::init_device(int width, int height)
{
    v4l2_capability cap;
    v4l2_cropcap cropcap;
    v4l2_crop crop;
    v4l2_format fmt;

    if(-1 == ioctl(fd, VIDIOC_QUERYCAP, &cap))
    {
        if(EINVAL == errno)
        {

        }
        else
        {

        }
        perror("VIDIOC_QUERYCAP");
        return -1;
    }

    if(!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE))
    {
        perror("V4L2_CAP_VIDEO_CAPTURE");
        return -1;
    }

    if(!(cap.capabilities & V4L2_CAP_STREAMING))
    {
        perror("V4L2_CAP_STREAMING");
        return -1;
    }

    CLEAR(cropcap);

    cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if(0 == ioctl(fd, VIDIOC_CROPCAP, &cropcap))
    {
        CLEAR(crop);
        crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        crop.c = cropcap.defrect;

        if(-1 == ioctl(fd, VIDIOC_S_CROP, &crop))
        {
            if(EINVAL == errno)
            {

            }
            else
            {
                perror("VIDIOC_S_CROP");
                return -1;
            }
        }
    }
    else
    {
        perror("VIDIOC_CROPCAP");
        return -1;
    }

    CLEAR(fmt);

    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = width;
    fmt.fmt.pix.height = height;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    fmt.fmt.pix.field = V4L2_FIELD_ANY;//V4L2_FIELD_INTERLACED;

    if(-1 == ioctl(fd, VIDIOC_S_FMT, &fmt))
    {
        perror("VIDIOC_S_FMT");
        return -1;
    }

    if(-1 == init_mmap())
    {
        perror("init_mmap");
        return -1;
    }

    ioctl(fd, VIDIOC_G_FMT, &fmt);

    return 0;
}

int VideoDevice::init_mmap()
{
    v4l2_requestbuffers req;
    CLEAR(req);

    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if(-1 == ioctl(fd, VIDIOC_REQBUFS, &req))
    {
        if(EINVAL == errno)
        {
            perror("VIDIOC_REQBUFS");
            return -1;
        }
        else
        {
            perror("VIDIOC_REQBUFS");
            return -1;
        }
    }

    if(req.count < 2)
    {
        perror("VIDIOC_REQBUFS");
        return -1;
    }

    buffers = (buffer*)calloc(req.count, sizeof(*buffers));

    if(!buffers)
    {

        return -1;
    }

    for(n_buffers = 0; n_buffers < req.count; ++n_buffers)
    {
        v4l2_buffer buf;
        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = n_buffers;

        if(-1 == ioctl(fd, VIDIOC_QUERYBUF, &buf))
        {
            perror("VIDIOC_QUERYBUF");
            return -1;
        }

        buffers[n_buffers].length = buf.length;
        buffers[n_buffers].start =
                mmap(NULL, // start anywhere
                     buf.length,
                     PROT_READ | PROT_WRITE,
                     MAP_SHARED,
                     fd, buf.m.offset);

        if(MAP_FAILED == buffers[n_buffers].start)
        {
            perror("MAP_FAILED");
            return -1;
        }
    }
    return 0;

}

int VideoDevice::start_capturing()
{
    unsigned int i;
    for(i = 0; i < n_buffers; ++i)
    {
        v4l2_buffer buf;
        CLEAR(buf);

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory =V4L2_MEMORY_MMAP;
        buf.index = i;

        if(-1 == ioctl(fd, VIDIOC_QBUF, &buf))
        {

            return -1;
        }
    }

    v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if(-1 == ioctl(fd, VIDIOC_STREAMON, &type))
    {

        return -1;
    }

    FD_ZERO(&fds);
    FD_SET(fd,&fds);
    tv.tv_sec = 2;
    tv.tv_usec = 0;
    if(0 == select(fd+1,&fds,NULL,NULL,&tv))
    {

        return -1;
    }
    return 0;
}

int VideoDevice::stop_capturing()
{
    v4l2_buf_type type;
    type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

    if(-1 == ioctl(fd, VIDIOC_STREAMOFF, &type))
    {

        return -1;
    }
    return 0;
}

int VideoDevice::uninit_device()
{
    unsigned int i;
    for(i = 0; i < n_buffers; ++i)
    {
        if(-1 == munmap(buffers[i].start, buffers[i].length))
        {

            return -1;
        }

    }
    free(buffers);
    return 0;
}

int VideoDevice::get_frame(void **frame_buf, size_t* len)
{

    fd_set fds;
    struct timeval tv;
    FD_ZERO (&fds);
    FD_SET (fd, &fds);

    tv.tv_sec = 0; tv.tv_usec = 500000;// Timeout.
    int r = select (fd + 1, &fds, NULL, NULL, &tv);
    if(-1 == r){
        perror("select");
    };
    if(0 == r){
        fprintf(stderr, "select timeout\n");
        return -1;
    };

    v4l2_buffer queue_buf;
    CLEAR(queue_buf);

    queue_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    queue_buf.memory = V4L2_MEMORY_MMAP;

    if(-1 == ioctl(fd, VIDIOC_DQBUF, &queue_buf))
    {
        switch(errno)
        {
        case EAGAIN:
            return -1;
        case EIO:
            return -1 ;
        default:
            return -1;
        }
    }

    *frame_buf = buffers[queue_buf.index].start;
    *len = buffers[queue_buf.index].length;
    index = queue_buf.index;

    return 0;

}

int VideoDevice::release_frame()
{
    if(index != -1)
    {
        v4l2_buffer queue_buf;
        CLEAR(queue_buf);

        queue_buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        queue_buf.memory = V4L2_MEMORY_MMAP;
        queue_buf.index = index;

        if(-1 == ioctl(fd, VIDIOC_QBUF, &queue_buf))
        {
            return -1;
        }
        return 0;
    }
    return -1;
}

int VideoDevice::get_framesize_type(int &count, struct frame_size **fms)
{

    int var;
    struct v4l2_frmsizeenum fmse;
    for (var = 0; ; var++) {
        fmse.index = var;
        fmse.pixel_format = V4L2_PIX_FMT_YUYV;
        int ret = ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fmse);
        if(ret==0){

        }else if (ret == -1 && var > 0) {
            break;
        }else {

            return -1;
        }
    }
    count = var;
    *fms = new struct frame_size[var];
    for (int i = 0; i < var; i++) {
        fmse.index = i;
        fmse.pixel_format = V4L2_PIX_FMT_YUYV;
        ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fmse);
        (*fms+i)->width = fmse.discrete.width;
        (*fms+i)->height = fmse.discrete.height;
    }
    return 0;
}

int VideoDevice::get_framesize(int *frame_width, int *frame_height,int *frequency)
{
    int ret;
    struct v4l2_fmtdesc fmt;
    fmt.index = 0;
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ret = ioctl(fd, VIDIOC_ENUM_FMT, &fmt);

    struct v4l2_frmsizeenum fmse;
    fmse.index = 0;
    fmse.pixel_format = fmt.pixelformat;
    ret = ioctl(fd, VIDIOC_ENUM_FRAMESIZES, &fmse);
    *frame_width = fmse.discrete.width;
    *frame_height = fmse.discrete.height;

    struct v4l2_frmivalenum frmival;
    frmival.index = 0;
    frmival.pixel_format = fmt.pixelformat;
    frmival.width = fmse.discrete.width;
    frmival.height = fmse.discrete.height;
    ret = ioctl(fd, VIDIOC_ENUM_FRAMEINTERVALS, &frmival);
    *frequency = frmival.discrete.denominator/frmival.discrete.numerator;
    return ret;
}


