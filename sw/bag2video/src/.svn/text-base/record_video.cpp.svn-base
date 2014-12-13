#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CompressedImage.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/CvBridge.h"
#include <string>
#include <opencv/cv.h>
#include <opencv/highgui.h>

//Subscribes to Image messages, converts to opencv image and constructs a video
int fps     = 15;  // or 30
CvVideoWriter *writer;
bool initialized;
ros::Time last_framerate_update;
ros::Time start_time;
int frame_counter = 0;
std::string out_file_name;

void do_init(const sensor_msgs::ImageConstPtr& img)
{
    int is_color;
    if(img->encoding.find("MONO") != std::string::npos)
        is_color = false;
    else
        is_color = true;

    //TODO: codec, fps, and name as node parameters, or smarter fps
    writer = cvCreateVideoWriter(
        out_file_name.c_str(),
        CV_FOURCC('D','I','V','3'),
        fps,
        cvSize(img->width,img->height), 
        is_color);

    start_time = ros::Time::now();
    last_framerate_update = start_time;
    initialized = true;
    std::cout << "done" << std::endl;
}

void image_handler(const sensor_msgs::ImageConstPtr& img)
    // sensor_msgs::Image* msg_ptr)	// Message pointer
{
    //initialize the VideoWriter and time stamps
    if(!initialized)
        do_init(img);

    //convert Image msg to IplImage
    sensor_msgs::CvBridge bridge_;
    IplImage *cv_image = NULL;
    try
    {
        cv_image = bridge_.imgMsgToCv(img, "bgr8");
    }
    catch (sensor_msgs::CvBridgeException error)
    {
        ROS_ERROR("error");
    }

    //write the frame to video and show a preview
    cvWriteFrame(writer,cv_image);     	    
    cvShowImage("record_video",cv_image);  
    //cvWaitKey(20);            		    // wait 20 ms (why?)

    frame_counter++;

    double delta_t = (ros::Time::now() - last_framerate_update).toSec();
    if(delta_t > 1.0)
    {
        double total_dur = (ros::Time::now() - start_time).toSec();
        std::cout << "\r" << frame_counter/delta_t << " fps, " 
                  << total_dur << " seconds" << std::flush;
        last_framerate_update = ros::Time::now();
        frame_counter = 0;
    }
}


int main(int argc, char **argv)
{
    std::string in_topic;
    initialized = false;

    ros::init(argc, argv, "image_listener");
    ros::NodeHandle nh;
    ros::NodeHandle nh_ns("~");

    nh_ns.param("outfile", out_file_name, std::string("out"));
    out_file_name = out_file_name + ".avi";
    nh_ns.param("in", in_topic, std::string("image"));
    std::cout << "waiting for first message on " << in_topic 
              << " topic to initialize..." << std::flush;

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(in_topic, 1, &image_handler);

    cvNamedWindow("record_video", CV_WINDOW_AUTOSIZE);

    ros::spin();                 
    cvReleaseVideoWriter(&writer);      //release video writer

    return 0;
}
