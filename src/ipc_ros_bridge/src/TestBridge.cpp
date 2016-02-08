
#include <ipc.h>
#include <Components/Controllers/CoordinatedController/coordinatedController-ipc.h>
#include <Components/Controllers/CoordinatedController/utils.h>
#include <Common/commonMath.h>
// #include <Common/ConstrainedMoveMessages.h>
#include <iostream>
#include "ros/ros.h"
#include "marsarm_moveit/Joints.h"

// struct Status
// {
//   Status () : calibrated(false), moveDone(false), touched(false) {}

//   NDofJointData jointAngles;
//   NDofJointData jointVels;
//   Pose eePose;
//   Pose eeGoalPose;
//   Pose eeEndPose;
//   Pose forceMM;
//   Pose forceNoise;
//   bool calibrated;
//   bool moveDone;
//   bool touched;
// };

static ros::Publisher pub;

static void forceSensorNoiseHnd (MSG_INSTANCE msg, void *callData,
				 void* clientData)
{
  // Status status;
  CoordinatedControllerStatus* ccStatus = (CoordinatedControllerStatus *)callData;
  std::cout <<"Received ipc message" << std::endl;
  if (ccStatus->manipStatus.cur.positionLen > 0){
      // std::cout <<"Received ipc message" << std::endl;
  //   status.jointAngles = ccStatus->manipStatus.cur.position[0];
    // std::cout <<"Joint 0" << status.jointAngles.data[0] << std::endl;
    // std::cout <<"Joint 0: " << ccStatus->manipStatus.cur.position[0].data[0] << std::endl;
    marsarm_moveit::Joints j;
    j.joints.resize(7);
    // ROS_INFO("size is %d", ccStatus->manipStatus.cur.position[0].data.size());
    for(int i=0; i<j.joints.size(); i++){
      
      j.joints[i] = ccStatus->manipStatus.cur.position[0].data[i];
      //physical parallel joints are inverted compared to model
      if(i%2){
	j.joints[i] = -j.joints[i];
      }
      //Joint 1 has a physical offset not present in model
      if(i==1){
	j.joints[i] = j.joints[i] - 3.1415/2;
      }
      ROS_INFO("Joint %d is %f", i, j.joints[i]);
    }
    pub.publish(j);
  }

  


  IPC_freeData (IPC_msgInstanceFormatter(msg), callData);
}

void ipcInit()
{
  IPC_connect("tactileLocalize");
  // IPC_subscribeData("force_sensor_noise", forceSensorNoiseHnd, NULL);
  IPC_subscribeData("coordinated control status message", forceSensorNoiseHnd, NULL);
  // IPC_subscribeData("ipc_ros_msg", forceSensorNoiseHnd, NULL);
  std::cout << "HI" << std::endl;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ipc_bridge");
  ros::NodeHandle n;
  pub = n.advertise<marsarm_moveit::Joints>("/actual_joints",10);
  ipcInit();

  // ros::Duration d(10);
  // while( ros::ok()){
  //   ROS_INFO("IPC bridge still running");
  //   d.sleep();
  // }
  while(1){
    IPC_listenWait(100);
    ros::spinOnce();
    // ros::Duration(3.0).sleep();
    // sleep(3);
  }
  ros::spin();
}
