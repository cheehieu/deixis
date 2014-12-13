// preprocessor directives
#include <fstream>
#include <iostream>
#include <list>
#include "bandit_msgs/JointArray.h"
#include "bandit_msgs/Params.h"
#include "geometry_msgs/Point.h"
#include "marker_finder/Marker.h"
#include "std_msgs/Empty.h"
#include "logger.h"
#include "mutex.h"
#include "pose.h"
#include "random.h"
#include "ros/ros.h"
#include "time.h"
#include "wiimote.h"
using namespace std;



// Bandit height (in inches) definitions
#define BANDIT_HEAD_HEIGHT              (18.75f)
#define BANDIT_EYE_HEIGHT               (15.75f)
#define BANDIT_SHOULDER_HEIGHT          ( 8.5f)
#define BANDIT_BODY_HEIGHT              (BANDIT_SHOULDER_HEIGHT)
#define BANDIT_SHOULDER_OFFSET          ( 6.5f)
#define BANDIT_SHOULDER_TO_ELBOW_OFFSET ( 6.5f)



// Pioneer (P2 and AT models) height (in inches) definitions
#define PIONEER_P2_HEIGHT        ( 9.5f)
#define PIONEER_AT_HEIGHT        (10.5f)
#define PIONEER_TO_BANDIT_OFFSET (18.75f)



// total robot (Bandit + mount + Pioneer base) height (in inches) definitions
const double ROBOT_HEAD_HEIGHT     =         BANDIT_HEAD_HEIGHT       +
                                             PIONEER_TO_BANDIT_OFFSET +
                                             PIONEER_AT_HEIGHT;
const double ROBOT_EYE_HEIGHT      =         BANDIT_EYE_HEIGHT        +
                                             PIONEER_TO_BANDIT_OFFSET +
                                             PIONEER_AT_HEIGHT;
const double ROBOT_SHOULDER_HEIGHT =         BANDIT_SHOULDER_HEIGHT   +
                                             PIONEER_TO_BANDIT_OFFSET +
                                             PIONEER_AT_HEIGHT;
const double ROBOT_BODY_HEIGHT     =         BANDIT_BODY_HEIGHT       +
                                             PIONEER_TO_BANDIT_OFFSET +
                                             PIONEER_AT_HEIGHT;
#define      ROBOT_SHOULDER_OFFSET          (BANDIT_SHOULDER_OFFSET)
#define      ROBOT_SHOULDER_TO_ELBOW_OFFSET (BANDIT_SHOULDER_TO_ELBOW_OFFSET)
#define      ROBOT_HEIGHT                   (ROBOT_EYE_HEIGHT)
#define      HUMAN_HEIGHT                   (ROBOT_HEIGHT)



// joint ID definitions
enum JointID
{
  JOINT_ID_HEAD_PITCH = 0,  // ID = 0
  JOINT_ID_HEAD_PAN,        // ID = 1
  JOINT_ID_L_SHOULDER_FB,   // ID = 2
  JOINT_ID_L_SHOULDER_IO,   // ID = 3
  JOINT_ID_L_ELBOW_TWIST,   // ID = 4
  JOINT_ID_L_ELBOW,         // ID = 5
  JOINT_ID_L_WRIST_TWIST,   // ID = 6
  JOINT_ID_L_WRIST_TILT,    // ID = 7
  JOINT_ID_L_HAND_GRAB,     // ID = 8
  JOINT_ID_R_SHOULDER_FB,   // ID = 9
  JOINT_ID_R_SHOULDER_IO,   // ID = 10
  JOINT_ID_R_ELBOW_TWIST,   // ID = 11
  JOINT_ID_R_ELBOW,         // ID = 12
  JOINT_ID_R_WRIST_TWIST,   // ID = 13
  JOINT_ID_R_WRIST_TILT,    // ID = 14
  JOINT_ID_R_HAND_GRAB,     // ID = 15
  JOINT_ID_EYEBROWS,        // ID = 16
  JOINT_ID_MOUTH_TOP,       // ID = 17
  JOINT_ID_MOUTH_BOTTOM     // ID = 18
};// JointID



// joint chain definitions
#define HEAD             (0)
#define ARM_LEFT         (1)
#define ARM_RIGHT        (2)
#define EYEBROWS         (3)
#define MOUTH            (4)



// joint chain offset definitions
#define JOINT_OFFSET_HEAD      (JOINT_ID_HEAD_PITCH)
#define JOINT_OFFSET_ARM_LEFT  (JOINT_ID_L_SHOULDER_FB)
#define JOINT_OFFSET_ARM_RIGHT (JOINT_ID_R_SHOULDER_FB)
#define JOINT_OFFSET_EYEBROWS  (JOINT_ID_EYEBROWS)
#define JOINT_OFFSET_MOUTH     (JOINT_ID_MOUTH_TOP)



// modality mask definitions
#define MODALITY_MASK_HEAD (0b00000001)
#define MODALITY_MASK_EYES (0b00000010)
#define MODALITY_MASK_ARMS (0b00000100)
#define MODALITY_MASK_BODY (0b00001000)



// deictic event list initialization definitions
const Pose ROBOT_POSES[] =
{
  //Pose(PositionVector(-30.0f, 0.0f, ROBOT_HEIGHT),   0.0f),
  //Pose(PositionVector(-60.0f, 0.0f, ROBOT_HEIGHT),   0.0f),
  //Pose(PositionVector(-90.0f, 0.0f, ROBOT_HEIGHT),   0.0f)
  Pose(PositionVector(-36.0f, 0.0f, ROBOT_HEIGHT),   0.0f)
};// ROBOT_POSES[]
const Pose HUMAN_POSES[] =
{
  //Pose(PositionVector( 30.0f, 0.0f, HUMAN_HEIGHT), 180.0f),
  //Pose(PositionVector( 60.0f, 0.0f, HUMAN_HEIGHT), 180.0f),
  //Pose(PositionVector( 90.0f, 0.0f, HUMAN_HEIGHT), 180.0f)
  Pose(PositionVector( 36.0f, 0.0f, HUMAN_HEIGHT), 180.0f)
};// HUMAN_POSES[]
const int MODALITIES[] =
{
  MODALITY_MASK_HEAD,
  MODALITY_MASK_ARMS,
  MODALITY_MASK_HEAD | MODALITY_MASK_ARMS
};// MODALITIES[]
/*
const double TARGETS_Y[] =  // in inches; left of screen, +; right of screen, -
{
  -2.5f * 18.0f,
  -1.5f * 18.0f,
  -0.5f * 18.0f,
   0.5f * 18.0f,
   1.5f * 18.0f,
   2.5f * 18.0f
};// TARGETS_Y[]
const double TARGETS_Z[] =  // in inches; top of screen, +; bottom of screen, -
{
  HUMAN_HEIGHT + -1.5f * 18.0f,
  HUMAN_HEIGHT + -1.0f * 18.0f,
  HUMAN_HEIGHT + -0.5f * 18.0f,
  HUMAN_HEIGHT +  0.5f * 18.0f,
  HUMAN_HEIGHT +  1.0f * 18.0f,
  HUMAN_HEIGHT +  1.5f * 18.0f
};// TARGETS_Z[]
*/



// sound file definitions
enum SoundIndex
{
  SOUND_INDEX_BEGIN = 0,
  SOUND_INDEX_NEXT_EVENT,
  SOUND_INDEX_PREV_EVENT,
  SOUND_INDEX_CHANGE_POSE,
  SOUND_INDEX_LOG_SUCCEEDED,
  SOUND_INDEX_LOG_FAILED,
  SOUND_INDEX_END
};// SoundIndex
#define N_SOUND_FILENAMES (7)
const string SOUND_FILENAMES[N_SOUND_FILENAMES] =
{
  "sounds/begin.wav",
  "sounds/next_event.wav",
  "sounds/prev_event.wav",
  "sounds/change_pose.wav",
  "sounds/log_succeeded.wav",
  "sounds/log_failed.wav",
  "sounds/end.wav"
};// SOUND_FILENAMES



// target definitions
struct Target: PositionVector, Mutex
{

  // <data members>
  PositionVector head_actual;
  PositionVector eyes_actual;
  PositionVector arms_actual;
  PositionVector body_actual;
  PositionVector perceived;
  std::string target_type;

  // <constructors>

  // 
  Target(const double dx = 0.0f,
         const double dy = 0.0f,
         const double dz = 0.0f)
    : PositionVector(dx, dy, dz), Mutex(),
      head_actual(PositionVector()),
      eyes_actual(PositionVector()),
      arms_actual(PositionVector()),
      body_actual(PositionVector()),
      perceived(PositionVector())
  {
  } // Target(const double, const double, const double)

  // 
  Target(const PositionVector &pv)
    : PositionVector(pv), Mutex(),
      head_actual(PositionVector()),
      eyes_actual(PositionVector()),
      arms_actual(PositionVector()),
      body_actual(PositionVector()),
      perceived(PositionVector())
  {
  } // Target(const PositionVector &)



  // <utility functions>

  //
  PositionVector getPerceivedError()
  {
    return perceived - *this;
  } // getPerceivedError()

  //
  PositionVector getHeadActualError()
  {
    return head_actual - *this;
  } // getHeadActualError()

  //
  PositionVector getEyesActualError()
  {
    return eyes_actual - *this;
  } // getEyesActualError()

  //
  PositionVector getArmsActualError()
  {
    return arms_actual - *this;
  } // getArmsActualError()

  //
  PositionVector getBodyActualError()
  {
    return body_actual - *this;
  } // getBodyActualError()

  //
  PositionVector getHeadPerceivedError()
  {
    return perceived - head_actual;
  } // getHeadPerceivedError()

  //
  PositionVector getEyesPerceivedError()
  {
    return perceived - eyes_actual;
  } // getEyesPerceivedError()

  //
  PositionVector getArmsPerceivedError()
  {
    return perceived - arms_actual;
  } // getArmsPerceivedError()

  //
  PositionVector getBodyPerceivedError()
  {
    return perceived - body_actual;
  } // getBodyPerceivedError()
};// Target



// target minimum/maximum definitions
#define N_TARGETS_PER_MODALITY (16)
const double TARGET_Y_MAX  =  54.0f;         // in inches
const double TARGET_Y_MIN  = -TARGET_Y_MAX;  // in inches
const double TARGET_Z_MAX  =  72.0f + 9.0f;  // in inches; with added offset
const double TARGET_Z_MIN  =  18.0f + 9.0f;  // in inches; with added offset



// calibration definitions
#define N_CALIB_TARGETS_PER_MODALITY         (8)
#define CALIB_MODALITY_HEAD_INDEX            (0)
#define CALIB_MODALITY_ARMS_INDEX            (1)
#define CALIB_MODALITY_ARMS_CROSS_BODY_INDEX (2)
const Target TARGET_CALIB_NEAR[] =
{
  Target(0.0f,  9.0f, HUMAN_HEIGHT + 6.0f),  // head
  Target(0.0f,  9.0f, HUMAN_HEIGHT + 6.0f),  // arm
  Target(0.0f, -9.0f, HUMAN_HEIGHT + 6.0f)   // arm (cross-body)
};// TARGET_CALIB_NEAR[]
const Target TARGET_CALIB_FAR[] =
{
  Target(0.0f, -27.0f, HUMAN_HEIGHT - 12.0f),  // head
  Target(0.0f,  27.0f, HUMAN_HEIGHT - 12.0f),  // arm
  Target(0.0f, -27.0f, HUMAN_HEIGHT - 12.0f)   // arm (cross-body)
};// TARGET_CALIB_FAR[]



// deictic event definitions
struct DeicticEvent: Mutex
{

  // <data members>
  int    index;
  Pose   robot_pose;
  Pose   human_pose;
  int    modality;
  Target target_pos;
  Time   t_start;
  Time   t_stop;
  Time   t_experiment_start;
  Time   t_experiment_stop;

  // <constructors>
  DeicticEvent()
    : index(-1),
      robot_pose(Pose()),
      human_pose(Pose()),
      modality(0),
      target_pos(Target())
  {
    t_start = t_stop = t_experiment_start = t_experiment_stop = getTime();
  } // DeicticEvent()

  // <utility functions>

  //
  DeicticEvent operator =(const DeicticEvent &de)
  {
    index          = de.index;
    robot_pose     = de.robot_pose;
    human_pose     = de.human_pose;
    modality       = de.modality;
    target_pos     = de.target_pos;
    t_start        = de.t_start;
    t_stop         = de.t_stop;
    return *this;
  } // =(const DeicticEvent &)

  //
  bool operator ==(const DeicticEvent &de)
  {
    return (index      == de.index)      &&
           (robot_pose == de.robot_pose) &&
           (human_pose == de.human_pose) &&
           (modality   == de.modality)   &&
           (target_pos == de.target_pos);
  } // ==(const DeicticEvent &)

  // <logger functions>

  //
  bool logHeader(ofstream &log_file)
  {
    if (!log_file.is_open()) return false;

    log_file << "event_index,";
    log_file << "robot_pose_x,";
    log_file << "robot_pose_y,";
    log_file << "robot_pose_z,";
    log_file << "robot_pose_th,";
    log_file << "human_pose_x,";
    log_file << "human_pose_y,";
    log_file << "human_pose_z,";
    log_file << "human_pose_th,";
    log_file << "modality,";
    log_file << "target_desired_x,";
    log_file << "target_desired_y,";
    log_file << "target_desired_z,";
    log_file << "target_head_actual_x,";
    log_file << "target_head_actual_y,";
    log_file << "target_head_actual_z,";
    log_file << "target_eyes_actual_x,";
    log_file << "target_eyes_actual_y,";
    log_file << "target_eyes_actual_z,";
    log_file << "target_arms_actual_x,";
    log_file << "target_arms_actual_y,";
    log_file << "target_arms_actual_z,";
    log_file << "target_body_actual_x,";
    log_file << "target_body_actual_y,";
    log_file << "target_body_actual_z,";
    log_file << "target_perceived_x,";
    log_file << "target_perceived_y,";
    log_file << "target_perceived_z,";
    log_file << "target_perceived_error_x,";
    log_file << "target_perceived_error_y,";
    log_file << "target_perceived_error_z,";
    log_file << "target_head_actual_error_x,";
    log_file << "target_head_actual_error_y,";
    log_file << "target_head_actual_error_z,";
    log_file << "target_eyes_actual_error_x,";
    log_file << "target_eyes_actual_error_y,";
    log_file << "target_eyes_actual_error_z,";
    log_file << "target_arms_actual_error_x,";
    log_file << "target_arms_actual_error_y,";
    log_file << "target_arms_actual_error_z,";
    log_file << "target_body_actual_error_x,";
    log_file << "target_body_actual_error_y,";
    log_file << "target_body_actual_error_z,";
    log_file << "target_head_perceived_error_x,";
    log_file << "target_head_perceived_error_y,";
    log_file << "target_head_perceived_error_z,";
    log_file << "target_eyes_perceived_error_x,";
    log_file << "target_eyes_perceived_error_y,";
    log_file << "target_eyes_perceived_error_z,";
    log_file << "target_arms_perceived_error_x,";
    log_file << "target_arms_perceived_error_y,";
    log_file << "target_arms_perceived_error_z,";
    log_file << "target_body_perceived_error_x,";
    log_file << "target_body_perceived_error_y,";
    log_file << "target_body_perceived_error_z,";
    // ... log each desired joint position...
    // ... log each actual joint position...
    log_file << "t_event_start,";
    log_file << "t_event_stop,";
    log_file << "t_experiment_start,";
    log_file << "t_experiment_stop";
    log_file << "point_type" << endl;

    return true;
  } // logHeader(ostream &)

  //
  bool log(ofstream &log_file)
  {
    if (!log_file.is_open()) return false;

    log_file << index                                     << ",";
    log_file << robot_pose.x                              << ",";
    log_file << robot_pose.y                              << ",";
    log_file << robot_pose.z                              << ",";
    log_file << degreesToRadians(robot_pose.getHeading()) << ",";
    log_file << human_pose.x                              << ",";
    log_file << human_pose.y                              << ",";
    log_file << human_pose.z                              << ",";
    log_file << degreesToRadians(human_pose.getHeading()) << ",";
    log_file << modality                                  << ",";
    log_file << target_pos.x                              << ",";
    log_file << target_pos.y                              << ",";
    log_file << target_pos.z                              << ",";
    log_file << target_pos.head_actual.x                  << ",";
    log_file << target_pos.head_actual.y                  << ",";
    log_file << target_pos.head_actual.z                  << ",";
    log_file << target_pos.eyes_actual.x                  << ",";
    log_file << target_pos.eyes_actual.y                  << ",";
    log_file << target_pos.eyes_actual.z                  << ",";
    log_file << target_pos.arms_actual.x                  << ",";
    log_file << target_pos.arms_actual.y                  << ",";
    log_file << target_pos.arms_actual.z                  << ",";
    log_file << target_pos.body_actual.x                  << ",";
    log_file << target_pos.body_actual.y                  << ",";
    log_file << target_pos.body_actual.z                  << ",";
    log_file << target_pos.perceived.x                    << ",";
    log_file << target_pos.perceived.y                    << ",";
    log_file << target_pos.perceived.z                    << ",";
    log_file << target_pos.getPerceivedError().x          << ",";
    log_file << target_pos.getPerceivedError().y          << ",";
    log_file << target_pos.getPerceivedError().z          << ",";
    log_file << target_pos.getHeadActualError().x         << ",";
    log_file << target_pos.getHeadActualError().y         << ",";
    log_file << target_pos.getHeadActualError().z         << ",";
    log_file << target_pos.getEyesActualError().x         << ",";
    log_file << target_pos.getEyesActualError().y         << ",";
    log_file << target_pos.getEyesActualError().z         << ",";
    log_file << target_pos.getArmsActualError().x         << ",";
    log_file << target_pos.getArmsActualError().y         << ",";
    log_file << target_pos.getArmsActualError().z         << ",";
    log_file << target_pos.getBodyActualError().x         << ",";
    log_file << target_pos.getBodyActualError().y         << ",";
    log_file << target_pos.getBodyActualError().z         << ",";
    log_file << target_pos.getHeadPerceivedError().x      << ",";
    log_file << target_pos.getHeadPerceivedError().y      << ",";
    log_file << target_pos.getHeadPerceivedError().z      << ",";
    log_file << target_pos.getEyesPerceivedError().x      << ",";
    log_file << target_pos.getEyesPerceivedError().y      << ",";
    log_file << target_pos.getEyesPerceivedError().z      << ",";
    log_file << target_pos.getArmsPerceivedError().x      << ",";
    log_file << target_pos.getArmsPerceivedError().y      << ",";
    log_file << target_pos.getArmsPerceivedError().z      << ",";
    log_file << target_pos.getBodyPerceivedError().x      << ",";
    log_file << target_pos.getBodyPerceivedError().y      << ",";
    log_file << target_pos.getBodyPerceivedError().z      << ",";
    // ... log each desired joint position...
    // ... log each actual joint position...
    log_file << t_start                                   << ",";
    log_file << t_stop                                    << ",";
    log_file << t_experiment_start                        << ",";
    log_file << t_experiment_stop                         << ",";
    log_file << target_pos.target_type                    << endl;

    return true;
  } // log(ostream &)
};// DeicticEvent
typedef list<DeicticEvent>      DeicticEvents;
typedef DeicticEvents::iterator DeicticEventIterator;
//struct DeicticEventIterator: DeicticEvents::iterator, Mutex
//{
//};// DeicticEventIterator



// global variables
DeicticEvents*        g_events         = NULL;
DeicticEventIterator* g_curr_event     = NULL;
Mutex*                g_mtx_curr_event = NULL;
Logger<DeicticEvent>* g_event_logger   = NULL;
bool*                 g_running        = NULL;
double*               g_angle_desired  = NULL;
Wiimote*              g_wiimote        = NULL;



// ROS global variables
ros::NodeHandle*               g_nh              = NULL;
bandit_msgs::Params::Response* g_joint_info      = NULL;
ros::Publisher*                g_joint_publisher = NULL;
bandit_msgs::JointArray*       g_joint_array     = NULL;



// function prototypes
bool publishJointArray();
bool clearJointArray();
bool pointAt(Pose src_pose, PositionVector dst_pos, char modality_mask);
bool headPointAt(Pose src_pose, PositionVector dst_pos);
bool eyesPointAt(Pose src_pose, PositionVector dst_pos);
bool armsPointAt(Pose src_pose, PositionVector dst_pos,
                 int  which_arm = ARM_RIGHT);
bool bodyPointAt(Pose src_pose, PositionVector dst_pos);
bool setInitPose();
bool setHeadPose(double head_pitch, double head_pan);
bool setArmPose(double shoulder_fb, double shoulder_io,
                double elbow_twist, double elbow,
                double wrist_twist, double wrist_tilt,
                double hand_grab,
                int    which_arm = ARM_RIGHT);
bool setEyebrowsPose(double eyebrows);
bool setMouthPose(double mouth_top, double mouth_bottom);
PositionVector getBodyTarget(Pose robot_pose);
PositionVector getArmsTarget(Pose robot_pose);
PositionVector getEyesTarget(Pose robot_pose);
PositionVector getHeadTarget(Pose robot_pose);
bool generateDeicticEvents(DeicticEvents &deictic_events);
bool printDeicticEvents(DeicticEvents &deictic_events,
                        ostream       &out = std::cout);
template <typename T> bool randomizeList(list<T>  &l,
                                         const int n_iterations = 1);
template <typename Iterator> Iterator getRandIter(Iterator iter,
                                                  ssize_t n = 1);
bool playSoundIndex(const SoundIndex sound_index, bool foreground = false);
void jointStateCallback(const bandit_msgs::JointArrayConstPtr &j);



// Bandit function prototypes
int    getNJoints();
string getJointName(const int id);
double getJointMin(const int id);
double getJointMax(const int id);
double getJointPos(const int id);



// executes main program code
int main(int argc, char** argv)
{
  cout << "+ main()\n";

  srand(time(NULL));

  // initialize deictic events
  /*DeicticEvents temp_events;
  if (!generateDeicticEvents(temp_events))
  {
    printf("Unable to generate deictic events list...\n");
    return 1;
  }
  return 0;*/

  // initialize Wiimote
  Wiimote wiimote;
  strcpy((char*)wiimote.addr, "00:22:AA:4F:49:83");
  if (connectWiimote(&wiimote) == 0)
  {
    printf("Connected to Wiimote!\n");
    //unsigned char rpt_mode = 0;
    //toggle_bit(rpt_mode, CWIID_RPT_IR);
    //cwiid_set_rpt_mode(wiimote.wiimote, rpt_mode);
    g_wiimote = &wiimote;
  }
  else
  {
    printf("Unable to connect to Wiimote...\n");
  }

  // initialize ROS
  ros::init(argc, argv, "deixis");
  ros::NodeHandle nh;
  ros::Rate       loop_rate(10);

  if (nh.ok()) g_nh = &nh;

  // initialize Bandit ROS node
  ros::Subscriber         joint_subscriber = nh.subscribe("joint_state", 0,
                                                          jointStateCallback);
  ros::Publisher          joint_publisher  =
    nh.advertise<bandit_msgs::JointArray>("joint_cmd", 1000);
  bandit_msgs::JointArray joint_array;
  g_joint_array = &joint_array;

  // wait until Bandit joint information is received
  bandit_msgs::Params::Request  joint_info_req;
  bandit_msgs::Params::Response joint_info_res;
  while (nh.ok())
  {
    if (ros::service::call("/params", joint_info_req, joint_info_res))
      break;

    printf(".");
    fflush(stdout);

    ros::spinOnce();
    loop_rate.sleep();
  }
  g_joint_info = &joint_info_res;

  // validate number of joints
  if (getNJoints() <= 0)
  {
    printf("Invalid number of joints %d...\n", getNJoints());
    return 1;
  }
  g_joint_publisher = &joint_publisher;
  g_angle_desired   = new double[getNJoints()];
  for (int i = 0, n = getNJoints(); i < n; ++i)
    g_angle_desired[i] = 0.0f;

  // initialize Bandit joint positions
  // note: this is very poorly written...
  ros::spinOnce();
  clearJointArray();
  setInitPose();
  publishJointArray();
  ros::spinOnce();
  clearJointArray();
  setInitPose();
  publishJointArray();
  ros::spinOnce();
  sleep(3);

  /*PositionVector test;
  while (nh.ok())
  {
    ros::spinOnce();

    cout << "\nEnter the (x, y, z) location to point at: ";
    cin  >> test.x >> test.y >> test.z;
    pointAt(Pose(PositionVector(0.0f, 0.0f, 0.0f), 0.0f),
            test, MODALITY_MASK_HEAD);
    cout << endl;

    //loop_rate.sleep();
  }
  return 0;*/

  // initialize deictic events
  //read target list from file
  DeicticEvents events;
  if (!generateDeicticEvents(events))
  {
    printf("Unable to generate deictic events list...\n");
    return 1;
  }
  DeicticEventIterator curr_event = events.begin();
  DeicticEventIterator prev_event = events.end();
  Mutex                mtx_curr_event;
  Logger<DeicticEvent> event_logger;
  Time                 t_experiment_start;
  Time                 t_experiment_stop;
  g_events         = &events;
  g_curr_event     = &curr_event;
  g_mtx_curr_event = &mtx_curr_event;
  if (event_logger.open()) g_event_logger = &event_logger;
  mtx_curr_event.lock();
  Pose           robot_pose = curr_event->robot_pose;
  PositionVector target_pos = curr_event->target_pos;
  int            modality   = curr_event->modality;
  mtx_curr_event.unlock();

  bool event_changed = false;

  // wait for input to start
  bool running = false;
  g_running    = &running;
  printf("\nPress the HOME button to begin...\n");
  while (!running) sleep(1);

  // start experiment
  mtx_curr_event.lock();
  t_experiment_start = getTime();
  event_logger.data.t_experiment_start = t_experiment_start;
  curr_event->t_experiment_start       = t_experiment_start;
  curr_event->t_start                  = t_experiment_start;
  event_logger.log();
  mtx_curr_event.unlock();

  // main execution loop
  playSoundIndex(SOUND_INDEX_BEGIN);
  bool done          = false;
  while ((!done) && (nh.ok()))
  {
    //system("clear");

    ros::spinOnce();

    mtx_curr_event.lock();
    if      (curr_event == events.end()) done          = true;
    else if (curr_event == prev_event)   event_changed = false;
    else
    {
      robot_pose    = curr_event->robot_pose;
      target_pos    = curr_event->target_pos;
      modality      = curr_event->modality;
      event_changed = true;

      if ((curr_event->index > 0) &&
          ((curr_event->robot_pose != prev_event->robot_pose) ||
           (curr_event->human_pose != prev_event->human_pose)))
      {
        running = false;
      }
    }
    if (!running)
    {
        cout << "Check the pose of the robot and the human...\n";
        playSoundIndex(SOUND_INDEX_CHANGE_POSE);
    }
    while (!running)
    {
      mtx_curr_event.unlock();
      ros::spinOnce();
      sleep(1);
      loop_rate.sleep();
      mtx_curr_event.lock();
    };
    prev_event = curr_event;
    mtx_curr_event.unlock();

    // point at the target position from the robot pose
    // using the parameterized modality
    if ((event_changed) && (running))
    {
      clearJointArray();
      setInitPose();
      publishJointArray();
      sleep(3);

      curr_event->t_experiment_start = t_experiment_start;
      curr_event->t_start            = getTime();
      clearJointArray();
      pointAt(robot_pose, target_pos, modality);
      publishJointArray();
      cout << endl;
    }

    //usleep(10000);
    loop_rate.sleep();
  }
  clearJointArray();
  setInitPose();
  publishJointArray();
  playSoundIndex(SOUND_INDEX_END, true);

  // stop experiment
  t_experiment_stop = getTime();
  event_logger.data.t_experiment_stop = t_experiment_stop;
  event_logger.log();

  event_logger.close();

  closeWiimote(&wiimote);  // disconnect from Wiimote

  delete [] g_angle_desired;

  cout << "- main()\n";
  return 0;
} // main(int, char**)



//
bool publishJointArray()
{
  if ((g_joint_publisher == NULL) || (g_joint_array == NULL)) return false;

  //
  if (g_angle_desired != NULL)
  {
    std::vector<bandit_msgs::Joint>::const_iterator joint_iter;
    for (joint_iter  = g_joint_array->joints.begin();
         joint_iter != g_joint_array->joints.end();
         ++joint_iter)
    {
      int id = joint_iter->id;
      if ((id >= 0) && (id < getNJoints()))
        g_angle_desired[id] = joint_iter->angle;
    }
  }
 
  // publish to subscribing nodes
  ROS_INFO("Publishing joint array...");
  g_joint_publisher->publish(*g_joint_array);

  return true;
} // publishJointArray()



//
bool clearJointArray()
{
  if (g_joint_array == NULL) return false;
  g_joint_array->joints.clear();
  return true;
} // clearJointArray()



//
// note: all modalities assume a 0-degree heading...
bool pointAt(Pose src_pose, PositionVector dst_pos, char modality_mask)
{
  cout << "+ pointAt()\n";

  // calculate the relationship vector from
  // the source pose to the destination position
  PositionVector src_to_dst = src_pose.getRelationshipTo(dst_pos);

  // display relationship vector information
  printf("src_pose:      (%.2f, %.2f, %.2f) | %.2f\n",
         src_pose.x, src_pose.y, src_pose.z, src_pose.getHeading());
  printf("dst_pos:       (%.2f, %.2f, %.2f)\n",
         dst_pos.x,  dst_pos.y,  dst_pos.z);
  printf("src->dst:      (%.2f, %.2f, %.2f)\n",
         src_to_dst.x, src_to_dst.y, src_to_dst.z);

  if (modality_mask & MODALITY_MASK_HEAD) headPointAt(src_pose, dst_pos);
  if (modality_mask & MODALITY_MASK_EYES) eyesPointAt(src_pose, dst_pos);
  if (modality_mask & MODALITY_MASK_ARMS) armsPointAt(src_pose, dst_pos);
  if (modality_mask & MODALITY_MASK_BODY) bodyPointAt(src_pose, dst_pos);
  cout << "- pointAt()\n";
  return modality_mask;
} // pointAt(Pose, PositionVector, char)



//
bool headPointAt(Pose src_pose, PositionVector dst_pos)
{
  //cout << "+ headPointAt()\n";

  bool ret_val = eyesPointAt(src_pose, dst_pos);

  //cout << "- headPointAt()\n";
  return ret_val;
} // headPointAt(Pose, PositionVector)



//
bool eyesPointAt(Pose src_pose, PositionVector dst_pos)
{
  //cout << "+ eyesPointAt()\n";

  if ((g_joint_publisher == NULL) || (g_joint_info == NULL)) return false;

  // calculate the relationship vector from
  // the source pose to the destination position
  PositionVector src_to_dst = src_pose.getRelationshipTo(dst_pos);

  // transform relationship vector to head frame
  PositionVector eyes_to_dst = src_to_dst;
  eyes_to_dst.z              = dst_pos.z - ROBOT_EYE_HEIGHT;

  // pitch and pan head (camera)
  double head_pitch = clip(atan2(eyes_to_dst.z,
                                 PositionVector(eyes_to_dst.x,
                                                eyes_to_dst.y).magnitude()),
                           degreesToRadians(getJointMin(JOINT_ID_HEAD_PITCH)),
                           degreesToRadians(getJointMax(JOINT_ID_HEAD_PITCH)));
  double head_pan   = clip(atan2(eyes_to_dst.y, eyes_to_dst.x),
                           degreesToRadians(getJointMin(JOINT_ID_HEAD_PAN)),
                           degreesToRadians(getJointMax(JOINT_ID_HEAD_PAN)));

  // display head pitch and pan
  printf("eyes->dst:     (%.2f, %.2f, %.2f)\n",
         eyes_to_dst.x, eyes_to_dst.y, eyes_to_dst.z);
  printf("eyes:          (%.2f, %.2f)\n",
         radiansToDegrees(head_pitch), radiansToDegrees(head_pan));

  bool ret_val = setHeadPose(head_pitch, head_pan);
  //cout << "- eyesPointAt()\n";
  return ret_val;
} // eyesPointAt(Pose, PositionVector)



//
bool armsPointAt(Pose src_pose, PositionVector dst_pos, int which_arm)
{
  //cout << "+ armsPointAt()\n";

  if ((g_joint_publisher == NULL) || (g_joint_info == NULL)) return false;

  int joint_offset = 0;
  switch (which_arm)
  {
    case ARM_RIGHT: joint_offset = JOINT_OFFSET_ARM_RIGHT; break;
    case ARM_LEFT:  joint_offset = JOINT_OFFSET_ARM_LEFT;  //break;
      printf("Currently limited to right arm functionality..\n");
    default:        return false;
  }

  // calculate the relationship vector from
  // the source pose to the destination position
  PositionVector src_to_dst = src_pose.getRelationshipTo(dst_pos);

  // transform relationship vector to shoulder frame
  PositionVector shoulder_to_dst = src_to_dst;
  double         sgn = sign(shoulder_to_dst.angle());
  shoulder_to_dst.z  = dst_pos.z - ROBOT_SHOULDER_HEIGHT;
  //shoulder_to_dst.y -= sgn * ROBOT_SHOULDER_OFFSET;  // for both arms...
  //shoulder_to_dst.y -= ROBOT_SHOULDER_OFFSET;        // for left arm...
  shoulder_to_dst.y += ROBOT_SHOULDER_OFFSET;        // for right arm...

  // shoulder angles (in radians)
  double shoulder_fb =
    clip(0.5f * PI + atan2(shoulder_to_dst.z,
                           PositionVector(shoulder_to_dst.x,
                                          shoulder_to_dst.y).magnitude()),
         degreesToRadians(getJointMin(joint_offset + 0)),
         degreesToRadians(getJointMax(joint_offset + 0)));
  double shoulder_io =  0.0f;

  if (shoulder_to_dst.angle() < 0.0f)  // right arm
  {

    // for both arms...
    //shoulder_io = -sgn *
    //               clip(sgn * atan2(shoulder_to_dst.y, shoulder_to_dst.x),
    //                    SHOULDER_IO_MIN_ANGLE, SHOULDER_IO_MAX_ANGLE);

    // for left arm...
    //shoulder_io = -clip(sgn * atan2(shoulder_to_dst.y, shoulder_to_dst.x),
    //                    SHOULDER_IO_MIN_ANGLE, SHOULDER_IO_MAX_ANGLE);

    // for right arm...
    shoulder_io = clip(sgn * atan2(shoulder_to_dst.y, shoulder_to_dst.x),
                       degreesToRadians(getJointMin(joint_offset + 1)),
                       degreesToRadians(getJointMax(joint_offset + 1)));
  }

  // transform shoulder frame to elbow frame
  // note: not sure if this actually works for shoulder IO + elbow movement...
  PositionVector elbow_to_dst = shoulder_to_dst.rotated(-shoulder_io);
  elbow_to_dst.x -= ROBOT_SHOULDER_TO_ELBOW_OFFSET * sin(shoulder_fb);
  elbow_to_dst.z += ROBOT_SHOULDER_TO_ELBOW_OFFSET * cos(shoulder_fb);
  elbow_to_dst.x  = PositionVector(elbow_to_dst.x,
                                   elbow_to_dst.z).magnitude();
  elbow_to_dst.z  = 0.0f;

  // elbow angles (in radians)
  double elbow_twist = -0.5f * PI;
  double elbow       =  clip(degreesToRadians(elbow_to_dst.angle()),
                             degreesToRadians(getJointMin(joint_offset + 3)),
                             degreesToRadians(getJointMax(joint_offset + 3)));

  // wrist angles (in radians and servo positions)
  double wrist_twist = 0.5f * PI;
  double wrist_tilt  = 0.0f;  // 0 angle for servo; modify to use angles
  double hand_grab   = 0.0f;  // 0 angle for servo; modify to use angles

  // display right shoulder forward/backward (FB) and inward/outward (IO)
  printf("shoulder->dst: (%.2f, %.2f, %.2f)\n",
         shoulder_to_dst.x, shoulder_to_dst.y, shoulder_to_dst.z);
  printf("elbow->dst:    (%.2f, %.2f, %.2f)\n",
         elbow_to_dst.x, elbow_to_dst.y, elbow_to_dst.z);
  printf("arms:          (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f)\n",
         radiansToDegrees(shoulder_fb), radiansToDegrees(shoulder_io),
         radiansToDegrees(elbow_twist), radiansToDegrees(elbow),
         radiansToDegrees(wrist_twist), wrist_tilt, hand_grab);

  // note: added gain to shoulder_fb/_io to account for control dead zones...
  bool ret_val = setArmPose(shoulder_fb +
                              sin(shoulder_fb) * degreesToRadians(5.0f),
                            shoulder_io/* +
                              cos(shoulder_fb) * degreesToRadians(5.0f)*/,
                            elbow_twist, elbow,
                            wrist_twist, wrist_tilt,
                            hand_grab, which_arm);
  //cout << "- armsPointAt()\n";
  return ret_val;
} // armsPointAt(Pose, PositionVector, int)



//
bool bodyPointAt(Pose src_pose, PositionVector dst_pos)
{
  //cout << "+ bodyPointAt()\n";

  // check for connection to Pioneer P2OS node?
  // ...

  // calculate the relationship vector from
  // the source pose to the destination position
  PositionVector src_to_dst = src_pose.getRelationshipTo(dst_pos);

  // transform relationship vector to body frame
  PositionVector body_to_dst = src_to_dst;
  body_to_dst.z              = 0.0f;

  // body heading
  double heading = degreesToRadians(body_to_dst.angle());

  // display head pan and pitch
  printf("body->dst:     (%.2f, %.2f, %.2f)\n",
         body_to_dst.x, body_to_dst.y, body_to_dst.z);
  printf("body:          (%.2f)\n",
         radiansToDegrees(heading));

  // set the Pioneer P2 heading
  // ...

  // publish to subscribing nodes
  //ROS_INFO("Publishing heading...");
  // ...

  //cout << "- bodyPointAt()\n";
  return true;
} // bodyPointAt(Pose, PositionVector)



//
bool setInitPose()
{
  if ((g_joint_publisher == NULL) || (g_joint_array == NULL)) return false;

  double joint_angles[] =
  {
    degreesToRadians(  0.0f),  // head pitch
    degreesToRadians(  0.0f),  // head pan
    degreesToRadians( 30.0f),  // left shoulder FB
    degreesToRadians( 20.0f),  // left shoulder IO
    degreesToRadians(-90.0f),  // left elbow twist
    degreesToRadians(  0.0f),  // left elbow tilt
    degreesToRadians( 90.0f),  // left wrist twist
    0.0f,                      // left wrist tilt
    0.0f,                      // left hand grab
    degreesToRadians( 30.0f),  // right shoulder FB
    degreesToRadians( 20.0f),  // right shoulder IO
    degreesToRadians(-90.0f),  // right elbow twist
    degreesToRadians(  0.0f),  // right elbow
    degreesToRadians( 90.0f),  // right wrist twist
    0.0f,                      // right wrist tilt
    0.0f,                      // right hand grab
    0.0f,                      // eyebrows
    0.0f,                      // mouth top
    0.0f                       // mouth bottom
  };// joint_angles[]

  // set joint ID's and angles
  //bandit_msgs::JointArray joint_array;
  bandit_msgs::Joint      joint;

  // set the ID and angle of each joint
  for (int i = 0; i < getNJoints(); ++i)
  {
    joint.id    = i;
    joint.angle = joint_angles[i];

    // note: this should be temporary; handles the left wrist/hand problem...
    if ((i == JOINT_ID_L_WRIST_TILT) || (i == JOINT_ID_L_HAND_GRAB)) continue;

    g_joint_array->joints.push_back(joint);  // add joint to joint array
  }

  // publish to subscribing nodes
  //ROS_INFO("Publishing joint array...");
  //g_joint_publisher->publish(joint_array);

  return true;
} // setInitPose()



//
bool setHeadPose(double head_pitch, double head_pan)
{
  if ((g_joint_publisher == NULL) || (g_joint_array == NULL)) return false;

  int joint_offset = JOINT_OFFSET_HEAD;

  // set joint ID's and angles
  //bandit_msgs::JointArray joint_array;
  bandit_msgs::Joint      joint;

  // set the head pitch ID and angle
  joint.id    = joint_offset + 0;
  joint.angle = head_pitch;
  g_joint_array->joints.push_back(joint);  // add head pitch to joint array

  // set the head pan ID and angle
  joint.id    = joint_offset + 1;
  joint.angle = head_pan;
  g_joint_array->joints.push_back(joint);  // add head pan to joint array

  // publish to subscribing nodes
  //ROS_INFO("Publishing joint array...");
  //g_joint_publisher->publish(joint_array);

  return true;
} // setHeadPose(double, double)



//
bool setArmPose(double shoulder_fb, double shoulder_io,
                double elbow_twist, double elbow,
                double wrist_twist, double wrist_tilt,
                double hand_grab,
                int    which_arm)
{
  if ((g_joint_publisher == NULL) || (g_joint_array == NULL)) return false;

  int joint_offset = 0;
  switch (which_arm)
  {
    case ARM_LEFT:  joint_offset = JOINT_OFFSET_ARM_LEFT;  break;
    case ARM_RIGHT: joint_offset = JOINT_OFFSET_ARM_RIGHT; break;
    default:        return false;
  }

  // set joint ID's and angles
  //bandit_msgs::JointArray joint_array;
  bandit_msgs::Joint      joint;

  // set the right shoulder FB ID and angle
  joint.id    = joint_offset + 0;
  joint.angle = shoulder_fb;
  g_joint_array->joints.push_back(joint);  // add shoulder FB to joint array

  // set the right shoulder IO ID and angle
  joint.id    = joint_offset + 1;
  joint.angle = shoulder_io;
  g_joint_array->joints.push_back(joint);  // add shoulder IO to joint array

  // set the right elbow twist ID and angle
  joint.id    = joint_offset + 2;
  joint.angle = elbow_twist;
  g_joint_array->joints.push_back(joint);  // add elbow twist to joint array

  // set the right elbow ID and angle
  joint.id    = joint_offset + 3;
  joint.angle = elbow;
  g_joint_array->joints.push_back(joint);  // add elbow to joint array

  // set the right wrist twist ID and angle
  joint.id    = joint_offset + 4;
  joint.angle = wrist_twist;
  g_joint_array->joints.push_back(joint);  // add wrist twist to joint array

  // set the right wrist tilt and angle
  joint.id    = joint_offset + 5;
  joint.angle = wrist_tilt;
  g_joint_array->joints.push_back(joint);  // add wrist tilt to joint array

  // set the right hand grab ID and angle
  joint.id    = joint_offset + 6;
  joint.angle = hand_grab;
  g_joint_array->joints.push_back(joint);  // add hand grab to joint array

  // publish to subscribing nodes
  //ROS_INFO("Publishing joint array...");
  //g_joint_publisher->publish(joint_array);

  return true;
} // setArmPose(int, double, double, double, double, double, double, double)



//
bool setEyebrowsPose(double eyebrows)
{
  if ((g_joint_publisher == NULL) || (g_joint_array == NULL)) return false;

  int joint_offset = JOINT_OFFSET_EYEBROWS;

  // set joint ID's and angles
  //bandit_msgs::JointArray joint_array;
  bandit_msgs::Joint      joint;

  // set the eyebrows ID and angle
  joint.id    = joint_offset + 0;
  joint.angle = eyebrows;
  g_joint_array->joints.push_back(joint);  // add eyebrows to joint array

  // publish to subscribing nodes
  //ROS_INFO("Publishing joint array...");
  //g_joint_publisher->publish(joint_array);

  return true;
} // setEyebrowsPose(double, double)



//
bool setMouthPose(double mouth_top, double mouth_bottom)
{
  if ((g_joint_publisher == NULL) || (g_joint_array == NULL)) return false;

  int joint_offset = JOINT_OFFSET_MOUTH;

  // set joint ID's and angles
  //bandit_msgs::JointArray joint_array;
  bandit_msgs::Joint      joint;

  // set the mouth top ID and angle
  joint.id    = joint_offset + 0;
  joint.angle = mouth_top;
  g_joint_array->joints.push_back(joint);  // add mouth top to joint array

  // set the mouth bottom ID and angle
  joint.id    = joint_offset + 1;
  joint.angle = mouth_bottom;
  g_joint_array->joints.push_back(joint);  // add mouth bottom to joint array

  // publish to subscribing nodes
  //ROS_INFO("Publishing joint array...");
  //g_joint_publisher->publish(joint_array);

  return true;
} // setMouthPose(double, double)



// calculate actual target from head frame
PositionVector getHeadTarget(Pose robot_pose)
{
  PositionVector target_head = getEyesTarget(robot_pose);
  //cout << "target_head = " << target_head << endl;

  return target_head;
} // getHeadTarget(Pose)



// calculate actual target from eyes frame
PositionVector getEyesTarget(Pose robot_pose)
{
  PositionVector target_eyes;
  target_eyes.x = 0.0f;
  target_eyes.y = robot_pose.y +
    -robot_pose.x *
    tan(degreesToRadians(getJointPos(JOINT_ID_HEAD_PAN) +
                         robot_pose.getHeading()));
  target_eyes.z = ROBOT_HEIGHT +
    PositionVector(-robot_pose.x, target_eyes.y).magnitude() *
    tan(degreesToRadians(getJointPos(JOINT_ID_HEAD_PITCH)));
  //cout << "target_eyes = " << target_eyes << endl;

  return target_eyes;
} // getEyesTarget(Pose)



// calculate actual target from arms frame
PositionVector getArmsTarget(Pose robot_pose)
{

  // calculate actual target from shoulder frame
  // note: these signs are only implemented for the right arm;
  //       also assumes a 0-degree heading...
  PositionVector shoulder_to_target;
  shoulder_to_target.x = -robot_pose.x;
  //shoulder_to_target.y =  robot_pose.y + ROBOT_SHOULDER_OFFSET;  // left
  shoulder_to_target.y =  robot_pose.y - ROBOT_SHOULDER_OFFSET;  // right
  shoulder_to_target.z = ROBOT_SHOULDER_HEIGHT;
  //cout << "shoulder_to_target = " << shoulder_to_target << endl;

  // note: right shoulder IO angle is negated for all calculations;
  //       this should be undone in the case of the left shoulder IO...
  PositionVector elbow_to_target;
  double         shoulder_to_elbow_x =
    ROBOT_SHOULDER_TO_ELBOW_OFFSET *
    sin(degreesToRadians( getJointPos(JOINT_ID_R_SHOULDER_FB))) *
    cos(degreesToRadians(-getJointPos(JOINT_ID_R_SHOULDER_IO)));
  elbow_to_target.x = shoulder_to_target.x - shoulder_to_elbow_x;
  elbow_to_target.y = shoulder_to_target.y + shoulder_to_elbow_x *
    tan(degreesToRadians(-getJointPos(JOINT_ID_R_SHOULDER_IO)));
  elbow_to_target.z = shoulder_to_target.z + shoulder_to_elbow_x *
    tan(degreesToRadians(getJointPos(JOINT_ID_R_SHOULDER_FB) - 90.0f));
  double         elbow_to_target_angle =
    degreesToRadians( getJointPos(JOINT_ID_R_ELBOW) +
                     -getJointPos(JOINT_ID_R_SHOULDER_IO));
  //cout << "elbow_to_target = " << elbow_to_target << endl;

  PositionVector target_arms;
  target_arms.x = 0.0f;
  target_arms.y = elbow_to_target.y +
                  elbow_to_target.x * tan(elbow_to_target_angle);
  target_arms.z = elbow_to_target.z +
                  elbow_to_target.x *
                  tan(degreesToRadians(
                      getJointPos(JOINT_ID_R_SHOULDER_FB) - 90.0f));
  //cout << "target_arms = " << target_arms << endl;

  return target_arms;
} // getArmsTarget(Pose)



// calculate actual target from body frame
PositionVector getBodyTarget(Pose robot_pose)
{
  PositionVector target_body;
  target_body.x = 0.0f;
  target_body.y = robot_pose.y +
    -robot_pose.x * tan(degreesToRadians(robot_pose.getHeading()));
  target_body.z = 0.0f;
  //cout << "target_body = " << target_body << endl;

  return target_body;
} // getBodyTarget(Pose)



//
bool generateDeicticEvents(DeicticEvents &deictic_events)
{
  list<Pose>   robot_poses(ROBOT_POSES,
                           ROBOT_POSES + sizeof(ROBOT_POSES) /
                             sizeof(Pose));
  list<Pose>   human_poses(HUMAN_POSES,
                           HUMAN_POSES + sizeof(HUMAN_POSES) /
                             sizeof(Pose));
  list<int>    modalities(MODALITIES,
                          MODALITIES + sizeof(MODALITIES) /
                            sizeof(int));
  /*list<double> targets_y(TARGETS_Y,
                         TARGETS_Y + sizeof(TARGETS_Y) /
                           sizeof(double));
  list<double> targets_z(TARGETS_Z,
                         TARGETS_Z + sizeof(TARGETS_Z) /
                           sizeof(double));*/
  list<double> targets_y;
  list<double> targets_z;

  if (targets_y.size() != targets_z.size()) return false;

  DeicticEvent deictic_event;

  randomizeList(robot_poses);
  for (list<Pose>::iterator rp_iter = robot_poses.begin();
       rp_iter != robot_poses.end();
       ++rp_iter)
  {
    deictic_event.robot_pose = *rp_iter;

    randomizeList(human_poses);
    for (list<Pose>::iterator hp_iter = human_poses.begin();
         hp_iter != human_poses.end();
         ++hp_iter)
    {
      deictic_event.human_pose = *hp_iter;

      DeicticEvents temp;
      randomizeList(modalities);
      for (list<int>::iterator m_iter = modalities.begin();
           m_iter != modalities.end();
           ++m_iter)
      {
        deictic_event.modality = *m_iter;

        //for each target that was read from the file add a deictic event


        // add calibration targets
        switch (deictic_event.modality)
        {
          case MODALITY_MASK_HEAD:
            for (int i = 0; i < N_CALIB_TARGETS_PER_MODALITY; ++i)
            {
              deictic_event.target_pos.set(
                TARGET_CALIB_NEAR[CALIB_MODALITY_HEAD_INDEX]);
              temp.push_back(deictic_event);
              deictic_event.target_pos.set(
                TARGET_CALIB_FAR[ CALIB_MODALITY_HEAD_INDEX]);
              deictic_event.target_pos.target_type = "CALIBRATION";
              temp.push_back(deictic_event);
            }
            break;
          case MODALITY_MASK_EYES: break;
          case MODALITY_MASK_ARMS:
            for (int i = 0; i < N_CALIB_TARGETS_PER_MODALITY; ++i)
            {

              // arm
              deictic_event.target_pos.set(
                TARGET_CALIB_NEAR[CALIB_MODALITY_ARMS_INDEX]);
              temp.push_back(deictic_event);
              deictic_event.target_pos.set(
                TARGET_CALIB_FAR[ CALIB_MODALITY_ARMS_INDEX]);
              temp.push_back(deictic_event);

              // arm (cross-body)
              deictic_event.target_pos.set(
                TARGET_CALIB_NEAR[CALIB_MODALITY_ARMS_CROSS_BODY_INDEX]);
              temp.push_back(deictic_event);
              deictic_event.target_pos.set(
                TARGET_CALIB_FAR[ CALIB_MODALITY_ARMS_CROSS_BODY_INDEX]);
              temp.push_back(deictic_event);
            }
            break;
          case MODALITY_MASK_BODY: break;
          default: break;
        }

//        // add random targets
//        double target_y = 0.0f;
//        double target_z = 0.0f;
//        if (deictic_event.modality == MODALITY_MASK_ARMS)
//          for (int i = 0; i < N_TARGETS_PER_MODALITY; ++i)
//          {
//
//            // arm
//            target_y = frand(0.0f,         TARGET_Y_MAX);
//            target_z = frand(TARGET_Z_MIN, TARGET_Z_MAX);
//            deictic_event.target_pos.set(0.0f, target_y, target_z);
//            temp.push_back(deictic_event);
//
//            // arm (cross-body)
//            target_y = frand(TARGET_Y_MIN, 0.0f);
//            target_z = frand(TARGET_Z_MIN, TARGET_Z_MAX);
//            deictic_event.target_pos.set(0.0f, target_y, target_z);
//            temp.push_back(deictic_event);
//          }
//        else
//          for (int i = 0; i < N_TARGETS_PER_MODALITY; ++i)
//          {
//            target_y = frand(TARGET_Y_MIN, TARGET_Y_MAX);
//            target_z = frand(TARGET_Z_MIN, TARGET_Z_MAX);
//            deictic_event.target_pos.set(0.0f, target_y, target_z);
//            temp.push_back(deictic_event);
//          }

        /*randomizeList(targets_y);
        randomizeList(targets_z);
        for (list<double>::iterator ty_iter = targets_y.begin(),
                                    tz_iter = targets_z.begin();
             (ty_iter != targets_y.end()) &&
             (tz_iter != targets_z.end());
             ++ty_iter,
             ++tz_iter)
        {
          deictic_event.target_pos.set(0.0f, *ty_iter, *tz_iter);
          temp.push_back(deictic_event);
        }*/
      }
      randomizeList(temp);
      deictic_events.splice(deictic_events.end(), temp);
    }
  }
  randomizeList(deictic_events);

  // index all of the deictic events
  int i = 0;
  for (DeicticEventIterator de_iter = deictic_events.begin();
       de_iter != deictic_events.end();
       ++de_iter)
    de_iter->index = i++;

  printDeicticEvents(deictic_events);

  return true;
} // generateDeicticEvents(DeicticEvents &)



// print deictic events to the parameterized output stream
bool printDeicticEvents(DeicticEvents &deictic_events,
                        ostream       &out)
{
  if (deictic_events.empty()) return false;
  printf("\n# of events: %d\n\n", int(deictic_events.size()));
  for (DeicticEventIterator de_iter = deictic_events.begin();
       de_iter != deictic_events.end();
       ++de_iter)
  {
    out << de_iter->index      << ". "
        << de_iter->robot_pose << " / "
        << de_iter->human_pose << " / "
        << ((de_iter->modality & MODALITY_MASK_HEAD) ? "H" : " ")
        << ((de_iter->modality & MODALITY_MASK_EYES) ? "E" : " ")
        << ((de_iter->modality & MODALITY_MASK_ARMS) ? "A" : " ")
        << ((de_iter->modality & MODALITY_MASK_BODY) ? "B" : " ")
        << " / " << de_iter->target_pos << endl;
  }
  printf("\n# of events: %d\n\n", int(deictic_events.size()));
  return true;
} // printDeicticEvents(DeicticEvents &, ostream &)



//
template <typename T> bool randomizeList(list<T> &l, const int n_iterations)
{
  if (l.empty()) return false;

  list<T> temp;
  for (int i = 0; i < n_iterations; ++i)
  {
    temp = l;
    l.clear();

    while (!temp.empty())
    {
      typename list<T>::iterator iter = getRandIter(temp.begin(),
                                                    irand(0, temp.size()));
      l.push_front(*iter);
      temp.erase(iter);
    }
  }

  return true;
} // randomizeList(list<t>)



//
template <typename Iterator> Iterator getRandIter(Iterator iter, ssize_t n)
{
  std::advance(iter, n);
  return iter;
} // getRandIter(Iterator, ssize_t n)



// handler for interrupt signals (e.g., Ctrl-C)
void terminate(int param)
{
  cout << "+ terminate()\n";
  if (g_wiimote != NULL)
  {
    closeWiimote(g_wiimote);
    g_wiimote = NULL;
  }
  cout << "- terminate()\n";
  exit(1);
} // terminate(int)



// returns the number of joints
int getNJoints()
{
  if (g_joint_info == NULL) return 0;
  return g_joint_info->pos.size();
} // getNJoints()



// returns the name of the joint with the parameterized ID
string getJointName(const int id)
{
  assert((id >= 0) && (id < getNJoints()));
  return g_joint_info->name[id];
} // getJointName(const int)



// returns the minimum position of the joint with the parameterized ID
double getJointMin(const int id)
{
  assert((id >= 0) && (id < getNJoints()));
  return g_joint_info->min[id];
} // getJointMin(const int)



// returns the maximum position of the joint with the parameterized ID
double getJointMax(const int id)
{
  assert((id >= 0) && (id < getNJoints()));
  return g_joint_info->max[id];
} // getJointMax(const int)



// returns the angle (in radians) of the joint with the parameterized ID
double getJointPos(const int id)
{
  assert((id >= 0) && (id < getNJoints()));
  return g_joint_info->pos[id];
} // getJointMin(const int)



//
bool playSoundIndex(const SoundIndex sound_index, bool foreground)
{
  int  index = sound_index;
  if ((index < 0) || (index > N_SOUND_FILENAMES)) return false;

  string sys_call = "mplayer " + SOUND_FILENAMES[index] + " >> /dev/null 2>&1";
  if (!foreground) sys_call += " &";
  return system(sys_call.c_str()) == 0;
} // playSoundIndex(const SoundIndex, bool)



//
void jointStateCallback(const bandit_msgs::JointArrayConstPtr &joints)
{
  int n_joints  = getNJoints();
  if (n_joints <= 0) return;

  system("clear");

  // iterate through all joints in the 'joint array'
  std::vector<bandit_msgs::Joint>::const_iterator joint_iter;
  for (joint_iter  = joints->joints.begin();
       joint_iter != joints->joints.end();
       ++joint_iter)
  {
    int id = joint_iter->id;
    if (id >= n_joints)
    {
      printf("Joint ID %d out of bounds [0, %d]", id, n_joints - 1);
      continue;
    }

    double angle_desired = 0.0f;
    if (g_angle_desired != NULL)
      angle_desired = radiansToDegrees(g_angle_desired[id]);
    double angle_actual  = radiansToDegrees(joint_iter->angle);
    g_joint_info->pos[id] = angle_actual;
    printf("[%d]: %.4f (%.4f)\n",
           id, g_joint_info->pos[id], angle_desired);
  }
  printf( "=====================\n");

  if ((g_curr_event != NULL) && (g_mtx_curr_event != NULL))
  {
    g_mtx_curr_event->lock();
    int            n_events    = ((g_events != NULL) ? g_events->size() : 0);
    int            event_index = (*g_curr_event)->index;
    Pose           robot_pose  = (*g_curr_event)->robot_pose;
    Pose           human_pose  = (*g_curr_event)->human_pose;
    PositionVector target_pos  = (*g_curr_event)->target_pos;
    int            modality    = (*g_curr_event)->modality;

    PositionVector target_head = getHeadTarget(robot_pose);
    PositionVector target_eyes = getEyesTarget(robot_pose);
    PositionVector target_arms = getArmsTarget(robot_pose);
    PositionVector target_body = getBodyTarget(robot_pose);

    (*g_curr_event)->target_pos.head_actual = target_head;
    (*g_curr_event)->target_pos.eyes_actual = target_eyes;
    (*g_curr_event)->target_pos.arms_actual = target_arms;
    (*g_curr_event)->target_pos.body_actual = target_body;

    PositionVector target_perceived = (*g_curr_event)->target_pos.perceived;

    Time           t_event_start    = (*g_curr_event)->t_start;
    Time           t_event_curr     = getTime();

    //double         t_event_start    =
    //  getTimeDiff((*g_curr_event)->t_start,
    //              (*g_curr_event)->t_experiment_start);

    //Time           t_now;
    //getTime(t_now);
    //double         t_event_curr     = t_now;
    //  getTimeDiff(t_now, (*g_curr_event)->t_experiment_start);

    //double         t_elapsed        = getTimeDiff(t_now,
    //                                              (*g_curr_event)->t_start);
    g_mtx_curr_event->unlock();

    printf("event_index        = %d of %d\n", event_index, n_events - 1);
    printf("robot_pose'        = [%.4f, %.4f, %.4f] | [%.4f]\n",
           -robot_pose.x, robot_pose.y, robot_pose.z, robot_pose.getHeading());
    printf("human_pose         = [%.4f, %.4f, %.4f] | [%.4f]\n",
           human_pose.x, human_pose.y, human_pose.z, human_pose.getHeading());
    printf("target_pos         = [%.4f, %.4f, %.4f]\n",
           target_pos.x, target_pos.y, target_pos.z);
    printf("target_head        = [%.4f, %.4f, %.4f]%s\n",
           target_head.x, target_head.y, target_head.z,
           ((modality & MODALITY_MASK_HEAD) ? "*" : ""));
    printf("target_eyes        = [%.4f, %.4f, %.4f]%s\n",
           target_eyes.x, target_eyes.y, target_eyes.z,
           ((modality & MODALITY_MASK_EYES) ? "*" : ""));
    printf("target_arms        = [%.4f, %.4f, %.4f]%s\n",
           target_arms.x, target_arms.y, target_arms.z,
           ((modality & MODALITY_MASK_ARMS) ? "*" : ""));
    printf("target_body        = [%.4f, %.4f, %.4f]%s\n",
           target_body.x, target_body.y, target_body.z,
           ((modality & MODALITY_MASK_BODY) ? "*" : ""));
    printf("target_perceived   = [%.4f, %.4f, %.4f]\n",
           target_perceived.x, target_perceived.y, target_perceived.z);
    printf("t_event_start|curr = [");
    cout << t_event_start << ", " << t_event_curr << "]\n";

    printf("=====================\n");
  }
} // jointStateCallback(const bandit_msgs::JointArrayConstPtr &)



// CWiid callback function
void cwiid_callback(cwiid_wiimote_t* wiimote,
                    int              mesg_count,
                    union cwiid_mesg mesg[],
                    timespec*        timestamp)
{
  jpwiimote* jpwm       = (jpwiimote*)wiimote;
  Wiimote*   wm         = (Wiimote*)jpwm->data;
  int        wiimote_id = atoi(&wm->label);
  double     seconds    = double(timestamp->tv_sec) +
                          double(timestamp->tv_nsec) / 1E9;

  static bool prepare_log_event = false;
  static bool confirm_log_event = false;

  for (int i = 0; i < mesg_count; ++i)
  {
    switch (mesg[i].type)
    {
      case CWIID_MESG_BTN:
        wm->updateButtons(mesg[i].btn_mesg.buttons);

        //if (wm->num_pressed > 0)
        {
          if (seconds - wm->time_pressed > 0.1f)
          {
            printf("Wiimote %d active!\n", wiimote_id);

            // start/continue running
            if ((wm->buttons[WMBTN_HOME].pressed) && (g_running != NULL))
            {
              *g_running = !(*g_running);
            }

            // ignore input if not running
            if ((g_running != NULL) && (!(*g_running)))
            {
              cout << "Change the pose of either the robot or the human...\n";
              playSoundIndex(SOUND_INDEX_CHANGE_POSE);
              continue;
            }

            // point at next target
            if ((wm->buttons[WMBTN_PLUS].pressed) &&
                (g_events         != NULL) &&
                (g_curr_event     != NULL) &&
                (g_mtx_curr_event != NULL))
            {
              g_mtx_curr_event->lock();
              if (*g_curr_event != g_events->end())
              {
                cout << "Point at next target...\n";
                playSoundIndex(SOUND_INDEX_NEXT_EVENT);
                std::advance(*g_curr_event, 1);
              }
              g_mtx_curr_event->unlock();
            }

            // point at previous target
            else if ((wm->buttons[WMBTN_MINUS].pressed) &&
                     (g_events         != NULL) &&
                     (g_curr_event     != NULL) &&
                     (g_mtx_curr_event != NULL))
            {
              g_mtx_curr_event->lock();
              if (*g_curr_event != g_events->begin())
              {
                cout << "Point at previous target...\n";
                playSoundIndex(SOUND_INDEX_PREV_EVENT);
                std::advance(*g_curr_event, -1);
              }
              g_mtx_curr_event->unlock();
            }

            // confirm logging data
            if ((wm->buttons[WMBTN_B].pressed) &&
                (wm->buttons[WMBTN_A].pressed) &&
                (prepare_log_event) && (!confirm_log_event))
            {
              confirm_log_event = true;
              //prepare_log_event = false;
              cout << "Confirming log data...\n";

              // wait until marker information is received
              marker_finder::Marker::Request  marker_finder_req;
              marker_finder::Marker::Response marker_finder_res;
              if ((ros::service::call("/get_marker_point",
                                     marker_finder_req,
                                     marker_finder_res)) &&
                  (marker_finder_res.count.data == 1)) // only 1 point found...
              {
                ROS_INFO("marker_finder_res: (%.2f, %.2f, %.2f)\n",
                         marker_finder_res.pt.x,
                         marker_finder_res.pt.y,
                         marker_finder_res.pt.z);
                playSoundIndex(SOUND_INDEX_LOG_SUCCEEDED);

                // set current perceived event data
                if ((g_curr_event != NULL) && (g_mtx_curr_event != NULL))
                {
                  g_mtx_curr_event->lock();
                  (*g_curr_event)->target_pos.perceived.set(
                    -marker_finder_res.pt.z,   // robot_x = -laser_z
                    -marker_finder_res.pt.x,   // robot_y = -laser_x
                     marker_finder_res.pt.y);  // robot_z =  laser_y
                  if (g_event_logger != NULL)
                  {
                    (*g_curr_event)->t_stop = getTime();
                    g_event_logger->data    = **g_curr_event;
                    g_event_logger->log();
                  }
                  g_mtx_curr_event->unlock();
                }
              }
              else
              {
                ROS_INFO("\ncount = %d\n", marker_finder_res.count.data);
                ROS_ERROR("Failed to call 'get_marker_point' service...\n");
                playSoundIndex(SOUND_INDEX_LOG_FAILED);
                /*if (g_event_logger != NULL)
                {
                  (*g_curr_event)->t_stop = getTime();
                  g_event_logger->data    = **g_curr_event;
                  g_event_logger->log();
                }*/
              }
            }
            else confirm_log_event = false;

            // prepare logging data
            if (( wm->buttons[WMBTN_A].pressed) &&
                //(!wm->buttons[WMBTN_B].pressed) &&
                (!prepare_log_event) && (!confirm_log_event))
            {
              prepare_log_event = true;
              confirm_log_event = false;
              cout << "Preparing log data...\n";
            }
            else
            {
              prepare_log_event = false;
              confirm_log_event = false;
            }
          }
          else printf("Detected wiimote %d double-tap...\n", wiimote_id);

          wm->time_pressed = seconds;
        }
        break;

      // Wiimote IR tracking; working code kept for future reference...
      /*case CWIID_MESG_IR:
        for (int j = 0; j < CWIID_IR_SRC_COUNT; ++j)
        {
          if (mesg[i].ir_mesg.src[j].valid)
          {
            printf("(%d, %d)\n",
                   mesg[i].ir_mesg.src[j].pos[CWIID_X],
                   mesg[i].ir_mesg.src[j].pos[CWIID_Y]);
          }
        }
        break;*/

      default: break;
    }
  }
} // void cwiid_callback(cwiid_wiimote_t*, int, cwiid_mesg [], timespec*)

