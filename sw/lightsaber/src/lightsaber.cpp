#include <ros/ros.h>
#include <string>
#include <joy/Joy.h>
#include <wiimote/RumbleControl.h>

ros::Time begin = ros::Time::now();
ros::Publisher wii_pub;
int count = 0;

enum SoundIndex {saber_on=0,saber,wookie,chewie,coin,wizard};
const std::string sounds[6] =
{
  "sounds/lightsaber_on.wav",
  "sounds/saber.wav",
  "sounds/wookie.wav",
  "sounds/chewie.wav",
  "sounds/log_succeeded.wav",
  "sounds/wizard_harry.wav"
};

bool playSound(SoundIndex i)
{
  std::string sys_call = "mplayer " + sounds[i] + " >> /dev/null 2>&1";
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
    if (joy->buttons[2]==1)     // pressed 'A'    
    {
        wiiRumble(-1,1,.4,0,0,0);             
        playSound(saber_on);
    }
    if(joy->buttons[3]==1)     //pressed 'B'     
        playSound(wizard);
    if(joy->buttons[4]==1)      //pressed '+'
        playSound(wookie);
    if(joy->buttons[5]==1)      //pressed '-'    
        playSound(chewie);

    if(joy->axes[0]==1)     //roll
    {
        //face up: 0
        //face right: 9.8
        //face down: 0
        //face left: -10.2
    }
    if(joy->axes[1] > 10 || joy->axes[2] > 20)     //pitch
    {
        //IR forward: 0 
        //IR up: -10.2
        //IR back: 0
        //IR down: 9.4
        wiiRumble(-1,1,.1,.1,0,0);   
        playSound(saber);
    }
    if(joy->axes[2]==1)     //yaw?
    {
        //IR forward: 9.8
        //IR up: 0
        //IR back: -8.6
        //IR down: 0
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lightsaber");
    ros::NodeHandle n;
    wii_pub = n.advertise<wiimote::RumbleControl>("wiimote/rumble",1);
    ros::Subscriber wii_sub = n.subscribe<joy::Joy>("joy", 10, &wiiCallback);
    std::cout << "\n\nWelcome, young Jedi...\n\n";
    ros::spin();
    return 0;
}
