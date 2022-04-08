/* Raw camera ROS publisher node for Arnie
 *
 * Author:
 * Jonathan Hanson
 *
 * Description:
 * The camera in Arnie is a shared resource; we cannot have two different
 * modules attempt to "own" the OpenCV VideoCapture object at the same time.
 * 
 * One solution to this problem is ROS! If we make a single ROS node that
 * opens and operates the VideoCapture device and publishes to a topic,
 * then any modules that desire to access camera frames can simply subscribe.
 * 
 * Using ROS is not _necessary_ to solve this problem, and you could make an
 * argument that it's overkill. But a big part of Arnie is learning how the
 * pieces of ROS interact, and this module is a simple way to look at how
 * images flow through the ROS ecosystem.
 * 
 * Usage:
 * Make sure to pass the index of the releveant /dev/videoX device!
 */
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>

int main(int argc, char** argv)
{
    // Ensure video source is passed as a parameter
    if(argv[1] == NULL) return 1;

    // Initialize the "camera" ROS node
    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;
    
    // Setup the image transport publisher
    // Note that we select queue_size=1; this is because we don't care if all
    // frames are consumed, and thus only have one in some "publish buffer" is
    // perfect.
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("frame", 1);

    // Parse the video device, ensuring it's a number
    std::istringstream video_sourceCmd(argv[1]);
    int video_source;
    if(!(video_sourceCmd >> video_source)) return 1;

    // Open the VideoCapture and check
    cv::VideoCapture cap(video_source);
    if(!cap.isOpened()) return 1;

    // Allocate our variables
    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    // Setup the camera to pub at 30 Hz
    ros::Rate loop_rate(30);

    // Main loop
    while (nh.ok()) {
        // Read a new frame
        cap >> frame;

        if(!frame.empty()) {
            // Use cv_bridge to convert cv::Mat to ROS sensor_msgs::msg::Image, then publish
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
            // cv::waitKey(1); //NOTE: this line is in the tutorial, but seems unncessary
        }
        ros::spinOnce(); //NOTE: necessary? does image_transport have some builtin callbacks?
        loop_rate.sleep();
    }
}
