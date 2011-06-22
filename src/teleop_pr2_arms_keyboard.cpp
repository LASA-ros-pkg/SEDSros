#include <termios.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65

class TeleopPR2Keyboard
{
  private:
  geometry_msgs::PoseStamped cmd;

  ros::NodeHandle n_;
  ros::Publisher pose_pub_;

  public:
  void init()
  {
    //header - this is impt
    cmd.header.frame_id = "/torso_lift_link";

    //Clear out our cmd - these values are roundabout initials

    // TODO -- read these from tf
    // rosrun tf tf_echo torso_lift_link r_gripper_tool_frame

    cmd.pose.position.x=0.9;
    cmd.pose.position.y=-0.186;
    cmd.pose.position.z=0.248;
    cmd.pose.orientation.x=-0.018;
    cmd.pose.orientation.y=-0.470;
    cmd.pose.orientation.z=0.001;
    cmd.pose.orientation.w=0.883;

    pose_pub_ = n_.advertise<geometry_msgs::PoseStamped>("r_cart/command_pose", 1);

    ros::NodeHandle n_private("~");
  }

  ~TeleopPR2Keyboard()   { }
  void keyboardLoop();
};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pr2_arms_keyboard");

  TeleopPR2Keyboard tpk;
  tpk.init();

  signal(SIGINT,quit);

  tpk.keyboardLoop();

  return(0);
}

void TeleopPR2Keyboard::keyboardLoop()
{
  char c;
  bool dirty=false;

  // get the console in raw mode
  tcgetattr(kfd, &cooked);
  memcpy(&raw, &cooked, sizeof(struct termios));
  raw.c_lflag &=~ (ICANON | ECHO);
  // Setting a new line, then end of file
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use 'WS' to forward/back");
  puts("Use 'AD' to left/right");
  puts("Use 'QE' to up/down");

  for(;;)
  {
    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    switch(c)
    {
      // Walking
    case KEYCODE_W:
      cmd.pose.position.x = cmd.pose.position.x+0.1;
      dirty = true;
      break;
    case KEYCODE_S:
      cmd.pose.position.x = cmd.pose.position.x-0.1;
      dirty = true;
      break;
    case KEYCODE_A:
      cmd.pose.position.y = cmd.pose.position.y+0.1;
      dirty = true;
      break;
    case KEYCODE_D:
      cmd.pose.position.y = cmd.pose.position.y-0.1;
      dirty = true;
      break;
    case KEYCODE_Q:
      cmd.pose.position.z = cmd.pose.position.z+0.1;
      dirty = true;
      break;
    case KEYCODE_E:
      cmd.pose.position.z = cmd.pose.position.z-0.1;
      dirty = true;
      break;

    }


    if (dirty == true)
    {
      pose_pub_.publish(cmd);
    }


  }
}
