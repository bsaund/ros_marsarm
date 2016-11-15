/*
 *  Utility for generating a random ray and pose for a measurement on the Boeing part
 *   for the Mars Arm.
 *
 */

#include "getRandomRayMarsArm.h"

#include <ros/ros.h>
#include "custom_ray_trace/plotRayUtils.h"
#include "custom_ray_trace/rayTracer.h"
#include "custom_ray_trace/rayTracePlotter.h"
#include <ros/console.h>
#include "getRandomRayMarsArm.h"
#include "stateMachine.h"





void generateRandomTouchWith(tf::Pose &probePose, double tbX, double tbY, double tbZ, double tbRR, double tbRP, double tbRY, double rotateR, double rotateP, double rotateY, double offX, double offY, double offZ)
{

  // tf::Transform rotate;
  // rotate.setRotation(tf::createQuaternionFromRPY(rotateR, rotateP, rotateY));
  tf::Pose touchBase;
  touchBase.setOrigin(tf::Vector3(tbX, tbY, tbZ));
  // std::cout << "RR " << tbRR << ", RP " << tbRP << ", RY " << tbRY << std::cout;
  tf::Quaternion q;
  q.setRPY(tbRR, tbRP, tbRY);

  touchBase.setRotation(q);

  // tf::Pose offset;
  // offset.setOrigin(tf::Vector3(offX, offY, offZ));
  // offset.setRotation(tf::createQuaternionFromRPY(0,0,0));

  // probePose = rotate*offset*touchBase;
  probePose = touchBase;

}

    // [0.6, 0.6, -0.1, 0, 0, 0]
void generateRandomTouchBottom(std::mt19937 &gen, tf::Pose &probePose)
{
  std::uniform_real_distribution<double> rand(0,1.0);

  if(rand(gen)> 0.5){
    //Generate bottom left touch
    double x_width = 0.6*rand(gen);
    double y_width = 0.45*rand(gen);

    // double y_val = -0.51 + y_width;
    double y_val = 0 + y_width;

    double yaw = -1.5 + 3*y_val;
    yaw = min(-0.8, yaw);
    // std::cout << "yaw: "<< yaw << "\n";
    // generateRandomTouchWith(probePose, 
    // 			  .53 + x_width, .4 + y_width, .687, M_PI, 0, 0, 
    // 			  0,0,0,
    // 			  0,0,0);
    generateRandomTouchWith(probePose, 
			    // 0.8, -0.21, 0.45, M_PI, 0, M_PI, 
			    0.7 + x_width, y_val, 0.45, M_PI, 0, yaw,
			    0,0,0,
			    0,0,0);
  }else{
    //Gereate right touch
    double x_width = 0.6*rand(gen);
    double y_width = 0.35*rand(gen);

    // double y_val = -0.51 + y_width;
    double y_val = -.45 + y_width;

    double yaw = -1.5 + 3*y_val;
    yaw = min(-0.8, yaw);
    yaw = max(-2.0, yaw);

    generateRandomTouchWith(probePose, 
			    // 0.8, -0.21, 0.45, M_PI, 0, M_PI, 
			    0.7 + x_width, y_val, 0.45, M_PI, 0, yaw,
			    0,0,0,
			    0,0,0);
  }
}

void generateRandomTouchFront(std::mt19937 &gen, tf::Pose &probePose)
{
  std::uniform_real_distribution<double> rand(-1.0,1.0);
  double y_width = 0.30*rand(gen);
  double z_width = 0.15*rand(gen);

  double yaw = 0;

  if(y_width < 0){
    yaw = 3*y_width;
  }

  generateRandomTouchWith(probePose, 
			  0.75, -0.2+y_width, 0.61+z_width, M_PI/2, yaw, -M_PI/2,
  			  // 0.812, -0.050, 0.391, -1.396, -2.104, -1.468, 
  			  0,0,0,
  			  0,0,0);
}



// void generateRandomTouchFrontRight(std::mt19937 &gen, tf::Pose &probePose)
// {
//   std::uniform_real_distribution<double> rand(-1.0,1.0);
//   double y_width = 0.10*rand(gen);
//   double z_width = 0.15*rand(gen);

//   generateRandomTouchWith(probePose, 
// 			  .577, -.661+y_width, .260+z_width, -1.5708, 0, -2.39,
//   			  0,0,0,
//   			  0,0,0);
// }


void generateRandomTouchSide(std::mt19937 &gen, tf::Pose &probePose)
{
  std::uniform_real_distribution<double> rand(-1.0,1.0);
  double x_width = 0.15*rand(gen);
  double z_width = 0.15*rand(gen);
  generateRandomTouchWith(probePose, 
			  .58+x_width, .20, .61+z_width, M_PI/2, 0, 0,
			  // .71, .13, .4,  2.724, -1.23, 1.58,
			  0,0,0,
			  0,0,0);

}



/*
 *  Generates a measurement ray from the set of rays reachable for the MarsArm robot. 
 *  These measurement vectors are tailored specifically for the robot and part 
 *  placement found in the CMU RI high bay.
 *  probePose, start, and end are all set by this function.
 */
void getRandomRayMarsArm(std::mt19937 &gen, tf::Pose &probePose, tf::Point &start, tf::Point &end)
{
  std::uniform_real_distribution<double> rand(0.0, 3.0);
  double faceNum = rand(gen);

  // generateRandomTouchSide(gen, probePose);


  // faceNum = 0.5; //HARDCODE FOR TESTING
  
  if(faceNum < 1.0)
    generateRandomTouchBottom(gen, probePose);
  else if(faceNum < 2.0)
    generateRandomTouchFront(gen, probePose);
  else
    generateRandomTouchSide(gen, probePose);
  
  // tf::Transform probeZ;
  // probeZ.setRotation(tf::createQuaternionFromRPY(0,0,0));
  // probeZ.setOrigin(tf::Vector3(0,0,0.1));


  // end = (probePose*probeZ).getOrigin();
  // start = tf::Point(0,0,0);
  // end = tf::Transform(probePose.getRotation()) * tf::Point(0,0,.1);


  start = probePose.getOrigin();
  end = probePose.getOrigin() + 
    tf::Transform(probePose.getRotation()) * tf::Point(0,0,-.25);
}



