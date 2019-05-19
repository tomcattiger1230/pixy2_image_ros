#include <pixy2_image_ros/pixy2_image_node.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pixy2_image");
    ros::NodeHandle nh;

    ros::NodeHandle nodeHandle("~");
    pixy2_ros::pixy2_image pixy2_image_obj(nodeHandle);
    ros::spin();
    return 0;
}