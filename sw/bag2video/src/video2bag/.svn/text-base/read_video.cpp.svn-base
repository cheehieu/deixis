#include <ros/ros.h>
#include <iostream>
#include <string>
#include <algorithm>
#include <vector>
#include <image_transport/image_transport.h>
#include <opencv/cvwimage.h>
#include <opencv/highgui.h>
#include <cv_bridge/CvBridge.h>
#include "boost/filesystem/operations.hpp"
#include "boost/filesystem/path.hpp"
#include "boost/progress.hpp"
#define BOOST_FILESYSTEM_NO_DEPRECATED

/************************************read_video.cpp*******************************************
o Reads all jpg files in a specified directory (argv[1])
o Sorts files numerically by name
o Publishes ros::Image messages at specified rate (argv[2]), default 15Hz, to video2bag/image topic
*NOTE: Images must be ".jpg", not .jpeg, .JPG, etc.
**********************************************************************************************/

namespace fs = boost::filesystem;
std::vector<std::string> str_vector;
std::string filename;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("video2bag/image", 1);
    ros::Rate loop_rate = (argc == 3) ? ros::Rate (atof(argv[2])) : ros::Rate (15);

    boost::progress_timer t(std::clog);
    fs::path full_path(fs::initial_path<fs::path>());
    if (argc > 1)
        full_path = fs::system_complete(fs::path( argv[1] ));
    else
        std::cout << "\nusage:   simple_ls [path]" << std::endl;

    unsigned long file_count = 0;

    if (!fs::exists( full_path ))
    {
        std::cout << "\nNot found: " << full_path.file_string() << std::endl;
        return 1;
    }

    if (fs::is_directory( full_path ))
    {
        std::cout << "\nIn directory: "
                  << full_path.directory_string() << "\n\n";
        fs::directory_iterator end_iter;
        for (fs::directory_iterator dir_itr( full_path );
            dir_itr != end_iter;
            ++dir_itr)
        {
            if (fs::is_regular_file( dir_itr->status() ))
            {
                filename = dir_itr->path().filename();
                str_vector.push_back(filename);               
                ++file_count;
            }
        }
    }
    else // must be a file
        std::cout << "\nFound: " << full_path.file_string() << "\n";    

    sort(str_vector.begin(),str_vector.end());		//sort vector alpha-numerically

    while (nh.ok()) 
    {
        for(unsigned long i=0 ; i < file_count ; i++)  
        {
            std::string file_path = full_path.directory_string() + str_vector[i];
            fs::path p(file_path.c_str(),fs::native);
            size_t dot = str_vector[i].find(".");
            if ((fs::file_size(p) != 0) && (str_vector[i].substr(dot)==".jpg"))  //checks if jpg is valid
            {
                if(i%1000==0)
                {
//                    std::cout << file_path << std::endl;
                    float percent = i*100;
                    std::cout << percent/file_count << "% complete\n";
                }
                cv::WImageBuffer3_b image( cvLoadImage(file_path.c_str(), CV_LOAD_IMAGE_COLOR) );
                sensor_msgs::ImagePtr msg = sensor_msgs::CvBridge::cvToImgMsg(image.Ipl(), "bgr8");
                pub.publish(msg);
                ros::spinOnce();
//                loop_rate.sleep();
            }
            else    //skip frame
            {
                ros::spinOnce();
//                loop_rate.sleep();
            }
        }
        exit(1);        //terminates after iterating through all jpgs
    }
    return 0;
}
