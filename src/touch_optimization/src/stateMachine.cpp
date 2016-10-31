#include "stateMachine.h"


#define MOTION_PARAM_NAME "/robot_motion_finished"

bool MotionStateMachine::isMotionFinished(ros::NodeHandle n){
  bool motionFinished;
  if(!n.getParam(MOTION_PARAM_NAME, motionFinished)){
    return true;
  }
  return motionFinished;
}

void MotionStateMachine::setMotionFinished(ros::NodeHandle n, bool finished){
  n.setParam(MOTION_PARAM_NAME, finished);
}
