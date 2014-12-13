#include <ros/ros.h>
#include <fstream>
#include <string>
#include <joy/Joy.h>
#include <marker_finder/Marker.h>
#include <wiimote/RumbleControl.h>

/******************************************************************
 * Subscribes to wiimote buttons,
 * Calls service /get_marker_point in deixis/main.cpp,
 * Records coordinates to output.txt file in CSV-like format.
 * 'A' to record one marker point
 * 'B' to see how many points are recorded (vibrations = # points)
 * 'HOME' to read output.txt file and print coordinates
 * '-' to close output.txt file and exit
 ******************************************************************/

std::fstream output;
ros::Time begin = ros::Time::now();
ros::Publisher wii_pub;
int rec_count;

bool playCoin()
{
	const std::string coin = "sounds/log_succeeded.wav";
	std::string sys_call = "mplayer " + coin + " >> /dev/null 2>&1";
	return system(sys_call.c_str()) == 0;
}

void wiiRumble(int sw_m,int cyc,float p0,float p1,float p2,float p3)
{
    wiimote::RumbleControl msg;
    msg.rumble.switch_mode = sw_m;          //set to -1=use_pattern, 1=ON, 0=OFF
    msg.rumble.num_cycles = cyc;            //repeat pulse_pattern
    msg.rumble.pulse_pattern.resize(4);     //define pulse_pattern array size
    msg.rumble.pulse_pattern[0] = p0;       //rumble time ON
    msg.rumble.pulse_pattern[1] = p1;       //rumble time OFF
    msg.rumble.pulse_pattern[2] = p2;       //rumble time ON
    msg.rumble.pulse_pattern[3] = p3;       //rumble time OFF
    wii_pub.publish(msg);                   //publish rumble message
}

void wiiCallback(const joy::Joy::ConstPtr& joy)
{
    if (joy->buttons[2]==1)     // pressed 'A' to record marker point coordinates
    {
        ros::Duration dur = ros::Time::now() - begin;
        begin = ros::Time::now();
        double secs = dur.toSec();
        if (secs>0.5)
        {
            marker_finder::Marker::Request marker_finder_req;
            marker_finder::Marker::Response marker_finder_res;
            if ((ros::service::call("/get_marker_point",marker_finder_req,marker_finder_res)) 
                                    && (marker_finder_res.count.data == 1)) //only 1 point found
            {
                output << -marker_finder_res.pt.z << ","    //robot_x = -laser_z
                       << -marker_finder_res.pt.x << ","    //robot_y = -laser_x
                       << marker_finder_res.pt.y << "\n";   //robot_z = laser_y 
                std::cout << "You just recorded a marker point.\n";
                wiiRumble(-1,1,.1,.1,0,0);    //'da' for active feedback
                playCoin();     //play coin sound
                rec_count++;
            }
            else
            {
                std::cout << "Error! Bad marker point.\n";
                wiiRumble(-1,1,2,0,0,0);    //long vibration for error feedback
            }
        }
    }
    if(joy->buttons[3]==1)     //pressed 'B' to display number of recorded points
    {
        ros::Duration dur = ros::Time::now() - begin;
        begin = ros::Time::now();
        double secs = dur.toSec();
        if (secs>0.5)
        {
            std::cout << "You recorded " << rec_count << " marker points.\n";
            wiiRumble(-1,rec_count,.03,0,0,.2);    //rumble rec_count times
        }
    }
    if(joy->buttons[10]==1)    //pressed 'HOME' to parse and print output.txt file
    {
        ros::Duration dur = ros::Time::now() - begin;
        begin = ros::Time::now();
        double secs = dur.toSec();
        int num = 0;
        output.close();
        if (secs>0.5)
        {
            std::ifstream input ("output.txt");
            std::string x,y,z;
            std::cout << "\n     *****MARKER POINT COORDINATES*****\n"    
                      << " -----------------------------------------\n"   
                      << "|  #  |  X-coord  |  Y-coord  |  Z-coord  |\n"
                      << " -----------------------------------------\n";
            while (num!=rec_count)
            {           
                getline(input,x,',');
                getline(input,y,',');
                getline(input,z);
                std::cout << "  " << ++num << "\t"
                          << x << "\t    " << y << "\t " << z << '\n';                
            }
            input.close();
        }
        output.open("output.txt",std::fstream::in | std::fstream::out | std::fstream::app);
    }
    if(joy->buttons[5]==1)      //pressed '-' to save text file and exit program
    {
        output.close();
        exit(1);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "marker_finder_record");
    ros::NodeHandle n;
    wii_pub = n.advertise<wiimote::RumbleControl>("wiimote/rumble",10);
    ros::Subscriber wii_sub = n.subscribe<joy::Joy>("joy", 10, &wiiCallback);
    
    output.open("output.txt",std::fstream::in | std::fstream::out | std::fstream::trunc);
    std::cout << "Press 'A' to record marker coordinates.\n";
    ros::spin();
    output.close();
    return 0;
}
