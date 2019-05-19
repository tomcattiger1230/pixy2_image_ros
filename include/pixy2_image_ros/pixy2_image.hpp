#ifndef _PIXY2_IMAGE_HPP_
#define _PIXY2_IMAGE_HPP_

#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <iostream>
#include <fstream>
#include <sstream>

// pixy2
#include <pixy2_image_ros/libpixyusb2.h>

namespace pixy2_ros{
    class pixy2_image
    {
        public:
            explicit pixy2_image(ros::NodeHandle nh);
            ~pixy2_image();

        private:
            void init(void);

            // ros node handle
            ros::NodeHandle nodeHandle_;
            image_transport::ImageTransport it;
            image_transport::Publisher pub;
            cv::Mat image_cv = cv::Mat(PIXY2_RAW_FRAME_HEIGHT, PIXY2_RAW_FRAME_WIDTH, CV_8UC3);

            // pixy object
            Pixy2 pixy;
            int initial_result; // not connection is -1, else is 0
            uint8_t *bayerFrame;
            sensor_msgs::Image sm_I;
            void inital_parameters_pixy2(uint8_t lamp=0, uint8_t fps=30);
            void vec2Mat(uint16_t width, uint16_t height, const uint8_t *bayerImage, cv::Mat& outpub_mat);

            // boost::thread pixy2_pub_thread;
    };
}

#endif