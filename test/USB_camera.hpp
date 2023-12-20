#ifndef __V4L2_CAPPTURE_HPP__
#define __V4L2_CAPPTURE_HPP__

#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <iostream>
#include <sys/mman.h>
#include <cstdint>
#include <cstring>
#include <stdlib.h>
#include <opencv2/opencv.hpp>


class USBCamera
{
public:
    USBCamera(const char *device, int width, int height, int fps) : device_(device), width_(width), height_(height), fps_(fps)
    {
    }

    bool openDevice()
    {
        fd_ = open(device_, O_RDWR);
        if (fd_ < 0)
        {
            std::cerr << "Failed to open device" << std::endl;
            return false;
        }
        return true;
    }

    bool queryCapability()
    {
        memset(&cap, 0, sizeof(cap));

        if (ioctl(fd_, VIDIOC_QUERYCAP, &cap) < 0)
        {
            printf("Get video capability error!\n");
            return false;
        }
        printf("driver : %s\n", cap.driver);
        printf("device : %s\n", cap.card);
        printf("bus : %s\n", cap.bus_info);
        printf("version : %d \n", cap.version);
        if (!(cap.device_caps & V4L2_BUF_TYPE_VIDEO_CAPTURE))
        {
            printf("Video device not support capture!\n");
            return false;
        }
        return true;
    }

    bool initDevice()
    {
        memset(&format, 0, sizeof(format));
        format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        format.fmt.pix.width = width_;
        format.fmt.pix.height = height_;
        format.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
        format.fmt.pix.field = V4L2_FIELD_ANY;

        if (ioctl(fd_, VIDIOC_S_FMT, &format) < 0)
        {
            std::cerr << "Failed to set format" << std::endl;
            return false;
        }
        printf("width = %d\n", format.fmt.pix_mp.width);
        printf("height = %d\n", format.fmt.pix_mp.height);
        printf("nmplane = %d\n", format.fmt.pix_mp.num_planes);

        // TODO: More initialization (e.g., buffer allocation) goes here

        return true;
    }

    bool mmap_v4l2_buffer()
    {
        int buf_num = 4;
        req.count = buf_num;
        req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        req.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd_, VIDIOC_REQBUFS, &req) < 0)
        {
            printf("Reqbufs fail\n");
            return -1;
        }

        buffers.resize(req.count);
        for (int i = 0; i < req.count; i++)
        {
            struct v4l2_buffer buf = {0};
            buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
            buf.memory = V4L2_MEMORY_MMAP;
            buf.index = i;

            if (-1 == ioctl(fd_, VIDIOC_QUERYBUF, &buf))
            {
                perror("VIDIOC_QUERYBUF");
                exit(EXIT_FAILURE);
            }
            

            buffers[i].length = buf.length;
            buffers[i].start = mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd_, buf.m.offset);

            if (MAP_FAILED == buffers[i].start)
            {
                perror("mmap");
                exit(EXIT_FAILURE);
            }

            if (-1 == ioctl(fd_, VIDIOC_QBUF, &buf))
            {
                perror("VIDIOC_QBUF");
                exit(EXIT_FAILURE);
            }
        }
        return true;
    }

    bool startCapture()
    {
        // TODO: Start capturing frames
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(fd_, VIDIOC_STREAMON, &type) < 0)
        {
            printf("VIDIOC_STREAMON failed\n");
            return false;
        }
        return true;
    }

    cv::Mat nv12ToMat(uint8_t *nv12Data, int width, int height)
    {
        cv::Mat nv12Image(height + height / 2, width, CV_8UC1, (unsigned char *)nv12Data);
        cv::Mat bgrImage;
        cv::cvtColor(nv12Image, bgrImage, cv::COLOR_YUV2BGR_NV12);

        return bgrImage;
    }

    cv::Mat writeVideoFrame()
    {
        struct v4l2_buffer buf = {0};
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (-1 == ioctl(fd_, VIDIOC_DQBUF, &buf)) {
            perror("VIDIOC_DQBUF");
            exit(EXIT_FAILURE);
        }

        cv::Mat frame(cv::Size(width_, height_), CV_8UC2, buffers[buf.index].start);
        cv::Mat frame_rgb;
        cv::cvtColor(frame, frame_rgb, cv::COLOR_YUV2BGR_YUYV);

        if (-1 == ioctl(fd_, VIDIOC_QBUF, &buf)) {
            perror("VIDIOC_QBUF");
            exit(EXIT_FAILURE);
        }

        return frame_rgb;
    }

    void stopCapture()
    {
        // TODO: Stop capturing frames
        enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        if (ioctl(fd_, VIDIOC_STREAMOFF, &type) < 0)
            printf("VIDIOC_STREAMOFF fail\n");
    }

    bool unmap_v4l2_buffer()
    {
        for (auto& buf : buffers) {
            munmap(buf.start, buf.length);
        }
        return true;
    }

    ~USBCamera()
    {
        if (fd_ >= 0)
        {
            unmap_v4l2_buffer();
            close(fd_);
        }
    }

private:
    int fd_ = -1;
    const char *device_;
    int width_, height_, fps_;
    struct v4l2_capability cap;
    struct v4l2_format format;
    struct v4l2_requestbuffers req;
    FILE *file_fd_raw;
    struct Buffer {
        void* start;
        size_t length;
    };
    std::vector<Buffer> buffers;
};

#endif
