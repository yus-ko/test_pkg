#include <ros/ros.h>
#include <potbot_lib/Utility.h>
#include <test_pkg/NavigationAction.h>
#include <actionlib/client/simple_action_client.h>
#include <stdio.h>
#include <termios.h>
#include <fcntl.h>
#include <time.h>

struct termios save_settings;
void set_input_interactive(void)
{
  // set no-echo-back and non-blocking
  struct termios settings;
  tcgetattr(0, &save_settings);
  settings = save_settings;
  settings.c_lflag &= ~(ECHO | ICANON);
  settings.c_cc[VTIME] = 0;
  settings.c_cc[VMIN] = 1;
  tcsetattr(0, TCSANOW, &settings);
  fcntl(0, F_SETFL, O_NONBLOCK);
}

void reset_input(void)
{
  tcsetattr(0, TCSANOW, &save_settings);
}

typedef actionlib::SimpleActionClient<test_pkg::NavigationAction> Client;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "task_client");
  ros::NodeHandle nh;
  set_input_interactive();

  Client client("task", true);

  printf("a: send goal (in 1s)\n");
  printf("s: send goal (in 5s)\n");
  printf("c: cancel goal\n");
  printf("q: quit\n");

  int task_id = 0;
  bool initial_goal = false;
  ros::Rate loop_rate(2);
  while (ros::ok())
  {
    if (client.isServerConnected())
    {
      char c = getchar();
      if (c == 'a')
      {
        test_pkg::NavigationGoal goal;
        goal.goal_pose.pose = potbot_lib::utility::get_Pose(1,2,0,0,0,M_PI_4);
        client.sendGoal(goal);
        ROS_INFO("send goal");
        potbot_lib::utility::print_Pose(goal.goal_pose.pose);
        initial_goal = true;
      }
    //   else if (c == 's')
    //   {
    //     test_pkg::NavigationGoal goal;
    //     goal.task_id = task_id;
    //     task_id++;
    //     goal.duration = 5.0;
    //     client.sendGoal(goal);
    //     printf("publish goal id:%i, duration:%f\n", goal.task_id, goal.duration);
    //     initial_goal = true;
    //   }
      else if (c == 'c')
      {
        client.cancelGoal();
        printf("publish cancel\n");
      }
      else if (c == 'q')
      {
        break;
      }
      if (initial_goal)
        printf("Current State: %s\n", client.getState().toString().c_str());
    }
    fflush(stdout);
    ros::spinOnce();
    loop_rate.sleep();
  }
  reset_input();
  return 0;
}
