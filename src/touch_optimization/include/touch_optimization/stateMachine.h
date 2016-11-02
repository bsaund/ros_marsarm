#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H
#include <ros/ros.h>


namespace MotionStateMachine{
  bool isMotionFinished(ros::NodeHandle n);
  void setMotionFinished(ros::NodeHandle n, bool finished);
}

#endif
