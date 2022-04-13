/* Raw camera ROS subscriber node
 *
 * Author:
 * Jonathan Hanson
 *
 * Description:
 * This node is just a simple preview of what is on the "frame" topic, which
 * is published by the camera node. It's also a nice example of a C++ ROS
 * subscriber. For debug use only.
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::imshow("preview", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(30);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "preview");
    ros::NodeHandle nh;

    cv::namedWindow("preview");

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe("frame", 1, imageCallback);
    ros::spin();
    cv::destroyWindow("preview");
}
