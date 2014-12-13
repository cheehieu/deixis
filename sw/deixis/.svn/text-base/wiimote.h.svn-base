// preprocessor directives
#ifndef WIIMOTE_H
#define WIIMOTE_H
#include <cwiid.h>
#include <pthread.h>



// function definitions
#define toggle_bit(bf, b) \
  ((bf) = ((bf) & (b)) \
           ? ((bf) & ~(b)) \
           : ((bf) |  (b)))



// Bluetooth address length definitions
#define BTADDRLEN (20)



// Wiimote button array index definitions
#define WMBTN_A     ( 0)
#define WMBTN_B     ( 1)
#define WMBTN_MINUS ( 2)
#define WMBTN_PLUS  ( 3)
#define WMBTN_HOME  ( 4)
#define WMBTN_1     ( 5)
#define WMBTN_2     ( 6)
#define WMBTN_UP    ( 7)
#define WMBTN_DOWN  ( 8)
#define WMBTN_LEFT  ( 9)
#define WMBTN_RIGHT (10)
#define NUM_WMBTNS  (11)



// button bit flags
static unsigned int wmbtnflags[NUM_WMBTNS] =
{
  0x8, 0x4, 0x10, 0x1000, 0x80, 0x2, 0x1, 0x800, 0x400, 0x100, 0x200
};



// "Wiimote button" type definition
struct WiimoteButton
{
  unsigned int flag;     // Wiimote bit flag for button
  bool         pressed;  // true if button is being pressed, false otherwise
};// WiimoteButton



// "Wiimote" type definition
struct Wiimote
{

  // <data members>
  char             addr[BTADDRLEN];       // Bluetooth address (user-specified)
  char             label;                 // Wiimote label (user-specified)
  WiimoteButton    buttons[NUM_WMBTNS];   // state of all buttons on Wiimote
  void*            data;                  // user-defined data pointer
  int              pressed[NUM_WMBTNS];   // buttons pressed in message
  int              released[NUM_WMBTNS];  // buttons released in message
  int              num_pressed;           // # buttons pressed in message
  int              num_released;          // # buttons released in message
  double           time_pressed;          // time of message
  cwiid_wiimote_t* wiimote;               // CWiid Wiimote structure

  // <constructors>
  Wiimote(): num_pressed(0), num_released(0), time_pressed(0.0f)
  {
    addr[0] = 0;
    for (int i = 0; i < NUM_WMBTNS; ++i)
    {
      buttons[i].flag    = wmbtnflags[i];
      buttons[i].pressed = false;
    }
  } // Wiimote()

  // <destructors>
  ~Wiimote()
  {
  } // ~Wiimote()

  // <utility functions>
  void updateButtons(unsigned int btnflags)
  {

    // reset button pressed/released counter
    num_pressed  = 0;
    num_released = 0;

    // check all buttons for presses/releases
    for (int i = 0; i < NUM_WMBTNS; ++i)
    {
      if (btnflags & buttons[i].flag)
      {

	      // button press detected
	      if (!buttons[i].pressed)
        {
	        pressed[num_pressed] = i;
	        ++num_pressed;
	      }
        buttons[i].pressed = true;
      }
      else
      {

        // button release detected
        if (buttons[i].pressed)
        {
          released[num_released] = i;
          ++num_released;
        }
        buttons[i].pressed = false;
      }
    }
  } // updateButtons(unsigned int)
};// Wiimote



// "read/write status" enumerated type definition
enum rw_status
{
  RW_IDLE,
  RW_READ,
  RW_WRITE,
  RW_CANCEL
};// rw_status



// "Juan-defined (JP) Wiimote" type definition
struct jpwiimote
{
  int                    flags;
  int                    ctl_socket;
  int                    int_socket;
  pthread_t              router_thread;
  pthread_t              status_thread;
  pthread_t              mesg_callback_thread;
  int                    mesg_pipe[2];
  int                    status_pipe[2];
  int                    rw_pipe[2];
  struct cwiid_state     state;
  enum rw_status         rw_status;
  cwiid_mesg_callback_t* mesg_callback;
  pthread_mutex_t        state_mutex;
  pthread_mutex_t        rw_mutex;
  pthread_mutex_t        rpt_mutex;
  int                    id;
  const void*            data;
};// jpwiimote



// CWiid callback function (user-defined)
cwiid_mesg_callback_t cwiid_callback;
void cwiid_callback(cwiid_wiimote_t* wiimote,
                    int              mesg_count,
                    union cwiid_mesg mesg[],
                    struct timespec* timestamp);



// CWiid error function
cwiid_err_t err;
void err(cwiid_wiimote_t* wiimote, const char* s, va_list ap)
{
  if (wiimote) printf("%d:", cwiid_get_id(wiimote));
  else         printf("-1:");
  vprintf(s, ap);
  printf("\n");
} // err(cwiid_wiimote_t*, const char*, va_list)



// sets the report mode of the Wiimote
void set_rpt_mode(cwiid_wiimote_t* wiimote, unsigned char rpt_mode)
{
  if (cwiid_set_rpt_mode(wiimote, rpt_mode))
  {
    fprintf(stderr, "Error setting report mode\n");
  }
} // set_rpt_mode(cwiid_wiimote_t*, unsigned char)



// connect to the Wiimote
int connectWiimote(Wiimote* wm)
{
  bdaddr_t bdaddr;	// Bluetooth device address

  cwiid_set_err(err);

  // Bluetooth address
  if(wm->addr[0])
  {
    str2ba(wm->addr,&bdaddr);
  }
  else
  {
    bdaddr = *BDADDR_ANY;
  }

  // attempt to connect to the Wiimote
  printf("Put Wiimote in discoverable mode now (press 1+2)...\n");
  if (!(wm->wiimote = cwiid_open(&bdaddr, 0)))
  {
    fprintf(stderr, "Unable to connect to Wiimote\n");
    return -1;
  }
  else
  {

    // setup data pointer to Wiimote object
    ((struct jpwiimote*)wm->wiimote)->data = wm;
    printf("Wiimote connected id: %d\n", ((struct jpwiimote*)wm->wiimote)->id);
  }
  if (cwiid_set_mesg_callback(wm->wiimote, cwiid_callback))
  {
    fprintf(stderr, "Unable to set message callback\n");
    return -1;
  }

  unsigned char rpt_mode = 0;
  toggle_bit(rpt_mode, CWIID_RPT_BTN);
  set_rpt_mode(wm->wiimote, rpt_mode);

  if (cwiid_enable(wm->wiimote, CWIID_FLAG_MESG_IFC))
  {
    fprintf(stderr, "Error enabling messages\n");
    return -1;
  }

  return 0;
} // connectWiimote(Wiimote*)



// close the connection to the Wiimote
int closeWiimote(Wiimote* wm)
{
  if (cwiid_close(wm->wiimote))
  {
    fprintf(stderr, "Error on Wiimote disconnect\n");
    return -1;
  }

  return 0;
} // closeWiimote(Wiimote*)

#endif

