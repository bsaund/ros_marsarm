/*
 *  Returns the ray and measurement pose corresponding to the 
 *  measurement with the highest information gain from amoung the
 *  random [numTrials] rays sampled.
 *  Measurements are randomly sampled based on the Scenario.
 */

#include "getBestRandomRay.h"
#include <ros/ros.h>
#include <ros/console.h>
#include "getRandomRayMarsArm.h"


/**
 * Randomly chooses vectors, gets the Information Gain for each of 
 *  those vectors, and returns the ray (start and end) with the highest information gain
 */
void getBestRandomRay(RayTracePlotter &plt, tf::Pose &probePose, 
		      Scenario raySet, int numTrials, bool plotAll)
{
  // tf::Point best_start, best_end;

  double bestIG;
  bestIG = 0;
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<double> rand(-1.0,1.0);

  tf::Point best_start, best_end;


  for(int i=0; i<numTrials; i++){
    tf::Point start, end;
    tf::Pose probePoseTmp;

    // ROS_DEBUG("Start of loop");
    switch(raySet){
    case MarsArm:
      getRandomRayMarsArm(gen, probePoseTmp, start, end);
      break;
    }

    Ray measurement(start, end);
    double IG = plt.getIG(measurement, 0.01, 0.002);
    

    if(plotAll)
      plt.plotRay(measurement, i);


    if (IG > bestIG){
      bestIG = IG;
      best_start = start;
      best_end = end;
      probePose = probePoseTmp;
    }


    ROS_DEBUG_THROTTLE(10, "Calculating best point based on information gain: %d...", i);
  }

  //Messages don't always get receieved. Delete multiple times to ensure removal of all rays
  plt.deleteAll();
  plt.plotRay(Ray(best_start, best_end));
  plt.deleteAll();
  plt.plotRay(Ray(best_start, best_end));
  ros::Duration(0.3).sleep();
  plt.plotRay(Ray(best_start, best_end));
  plt.plotIntersections(Ray(best_start, best_end));
  // plt.plotCylinder(best_start, best_end, 0.01, 0.002, true);
  ROS_INFO("Ray is: %f, %f, %f.  %f, %f, %f", 
  	   best_start.getX(), best_start.getY(), best_start.getZ(),
  	   best_end.getX(), best_end.getY(), best_end.getZ());
  ROS_INFO("IG is: %f", bestIG);
  
}
