#include <pixy2_image_ros/pixy2_image.hpp>

namespace pixy2_ros
{
    pixy2_image::pixy2_image(ros::NodeHandle nh):
    nodeHandle_(nh),
    it(nh)
    {
        ROS_INFO("[Pixy2 Image] Node started.");

        init();
    }

    pixy2_image::~pixy2_image()
    {

    }

    void pixy2_image::init(void)
    {
        initial_result = pixy.init();
        int lamp_level = 0;
        int fps_s = 0; 
        ros::param::get("pixy2_test/lamp", lamp_level);
        ros::param::get("pixy2_test/fps", fps_s);
        inital_parameters_pixy2(lamp_level, fps_s);
        if(initial_result<0)
        {
            ROS_ERROR("Can not connect to the Pixy2");
        }
        else
        {
            pub = it.advertise("/pixy2/image_raw", 1);
        
            while(!ros::isShuttingDown())
            {
                // need to call stop() befroe calling getRawFrame().
                // Note, you can call getRawFrame multiple times after calling stop().
                // That is, you don't need to call stop() each time.
                pixy.m_link.stop();
                // grab raw frame, BGGR Bayer format, 1 byte per pixel
                pixy.m_link.getRawFrame(&bayerFrame);
                // save bggr bayer data to the cv matrix
                vec2Mat(PIXY2_RAW_FRAME_WIDTH, PIXY2_RAW_FRAME_HEIGHT, bayerFrame, image_cv);
                

                sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image_cv).toImageMsg();
                pub.publish(msg);
            }
        }
        

    }

    void pixy2_image::vec2Mat(uint16_t width, uint16_t height, const uint8_t *bayerImage, cv::Mat& outpub_mat)
    {
        uint32_t x, y, xx, yy, r, g, b;
        uint8_t *pixel0, *pixel;
        
        for (y=0; y<height; y++)
        {
            yy = y;
            if (yy==0)
            yy++;
            else if (yy==height-1)
            yy--;
            pixel0 = (uint8_t *)bayerImage + yy*width;
            for (x=0; x<width; x++) //, image++
            {

            xx = x;
            if (xx==0)
            xx++;
            else if (xx==width-1)
            xx--;
            pixel = pixel0 + xx;
            if (yy&1)
            {
                if (xx&1)
                {
                    outpub_mat.at<cv::Vec3b>(y,x)[0] = *pixel;
                    
                    outpub_mat.at<cv::Vec3b>(y,x)[1] = (*(pixel-1)+*(pixel+1)+*(pixel+width)+*(pixel-width))>>2;
                    
                    outpub_mat.at<cv::Vec3b>(y,x)[2] = (*(pixel-width-1)+*(pixel-width+1)+*(pixel+width-1)+*(pixel+width+1))>>2;
                }
                else
                {
                    outpub_mat.at<cv::Vec3b>(y,x)[0] = (*(pixel-1)+*(pixel+1))>>1;
                    outpub_mat.at<cv::Vec3b>(y,x)[1] = *pixel;
                    outpub_mat.at<cv::Vec3b>(y,x)[2] = (*(pixel-width)+*(pixel+width))>>1;
                }
            }
            else
            {
                if (xx&1)
                {
                outpub_mat.at<cv::Vec3b>(y,x)[0] = (*(pixel-width)+*(pixel+width))>>1;
                outpub_mat.at<cv::Vec3b>(y,x)[1] = *pixel;
                outpub_mat.at<cv::Vec3b>(y,x)[2] = (*(pixel-1)+*(pixel+1))>>1;
                }
                else
                {
                outpub_mat.at<cv::Vec3b>(y,x)[0] = (*(pixel-width-1)+*(pixel-width+1)+*(pixel+width-1)+*(pixel+width+1))>>2;
                outpub_mat.at<cv::Vec3b>(y,x)[1] = (*(pixel-1)+*(pixel+1)+*(pixel+width)+*(pixel-width))>>2;
                outpub_mat.at<cv::Vec3b>(y,x)[2] = *pixel;
                }
            }
            }
        }


    }

    void pixy2_image::inital_parameters_pixy2(uint8_t lamp, uint8_t fps)
    {
        if (lamp==1)
        {
            pixy.setLamp(1,0);
        }
        else if (lamp==2)
        {
            pixy.setLamp(0,1);
        }
        else if (lamp==3)
        {
            pixy.setLamp(1,1);
        }
        else
        {
            pixy.setLamp(0,0);
        }


        
    }
} // namespace name
