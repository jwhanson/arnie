#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sstream>

int main(int argc, char** argv)
{
    //check if video source is passed as a parameter
    if(argv[1] == NULL) return 1;

    ros::init(argc, argv, "camera");
    ros::NodeHandle nh;
    
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("frame", 1);

    std::istringstream video_sourceCmd(argv[1]);
    int video_source;
    //ensure the source is indeed a number
    if(!(video_sourceCmd >> video_source)) return 1;

    cv::VideoCapture cap(video_source);
    //ensure video device can be opened
    if(!cap.isOpened()) return 1;

    cv::Mat frame;
    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(30);

    while (nh.ok()) {
        cap.read(frame); //NOTE: retrying with read; does this avoid queuing?
        if(!frame.empty()) {
            msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg();
            pub.publish(msg);
            cv::waitKey(1);
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}
