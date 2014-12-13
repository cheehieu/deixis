#include "ros/ros.h"
#include <string>
#include "rosrecord/Player.h"
#include "rosrecord/AnyMsg.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <opencv/cv.h>
#include <opencv/highgui.h>

//reads a bag file and publishes image messages using image_transport
//see http://www.ros.org/wiki/image_transport

//image_transport::ImageTransport it(nh);
//image_transport::Publisher pub = it.advertise<sensor_msgs::CompressedImage>("bagfile_image", 1);
sensor_msgs::CompressedImagePtr curImage;

// Callback for publisher
void image_handler(std::string name,      // Topic name       
                    sensor_msgs::CompressedImage* msg, // Message pointer  
                    ros::Time t,           // Shifted and scaled time     
                    ros::Time t_orig,      // Message timestamp
                    void* n)               // Void pointer    
//void image_handler(const sensor_msgs::ImageConstPtr& msg)
{
    //pub.publish(boost::shared_ptr<sensor_msgs::CompressedImage>(msg));
    std::cout << "setting image" << std::endl;
    sensor_msgs::CvBridge bridge_;
    IplImage *cv_image = NULL;
    try
    {
        boost::shared_ptr<sensor_msgs::Image> img_ptr(msg);
        cv_image = bridge_.imgMsgToCv(img_ptr, "bgr8");
        cvShowImage("mainWin",cv_image);   	// view the captured frames during capture
    }
    catch (sensor_msgs::CvBridgeException error)
    {
        ROS_ERROR("error");
    }
    //curImage = sensor_msgs::CompressedImagePtr(msg);
    std::cout << "/setting image" << std::endl;
}

int main(int argc, char **argv)
{
    ros::init(argc,argv, "image_publisher");
    ros::NodeHandle nh;  
    ros::Publisher pub;
    pub = nh.advertise<sensor_msgs::CompressedImage>("bagfile_compressed", 100);
    ros::record::Player player;
    std::string bagFileName; 

    cvNamedWindow("mainWin", CV_WINDOW_AUTOSIZE);

    if(argc == 2)
        bagFileName = std::string(argv[1]);
    else
        std::cout << "usage: read_bag <path to bag file with compressedimages>" 
                  << std::endl;

    if (player.open(bagFileName, ros::Time()))
    {
        player.addHandler<sensor_msgs::CompressedImage>(std::string("/Image"), 
                                                    &image_handler, NULL);
    } 
   
    // Spin until we have consumed the bag
    while(ros::ok()) //&& player.nextMsg())  
    {
//        int x;
//        std::cin >> x;
        std::cout << "publishing image" << std::endl;

        //pub.publish(curImage);
        std::cout << "/publishing image" << std::endl;

        ros::spinOnce();
    }
    return 0;
}
