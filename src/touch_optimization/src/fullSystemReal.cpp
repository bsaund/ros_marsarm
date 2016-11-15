/**
 *  This is a rosnode that randomly chooses touch points, 
 *    performs simulated measurements, and updates the particle filter using the
 *    MarsArm Robot at CMU RI.
 *   This node is made to work with a particle filter node, a node that 
 *    publishes visualization messages, and RViz.
 */

#include <ros/ros.h>
#include "particle_filter/PFilterInit.h"
#include "particle_filter/AddObservation.h"
#include "geometry_msgs/Point.h"
#include "std_msgs/String.h"
#include <tf/transform_broadcaster.h>
#include "custom_ray_trace/rayTracer.h"
#include "custom_ray_trace/rayTracePlotter.h"
#include <ros/console.h>
#include "getBestRandomRay.h"
#include "stateMachine.h"






geometry_msgs::Pose probeAt(tf::Transform rotate, tf::Transform base, double x, double y, double z, double r, double p, double yaw){
  tf::Pose offset;
  offset.setOrigin(tf::Vector3(x,y,z));
  offset.setRotation(tf::createQuaternionFromRPY(r, p, yaw));
  geometry_msgs::Pose probe_msg;
  tf::poseTFToMsg(rotate*offset*base, probe_msg);
  // probe_pub.publish(probe_msg);
  return probe_msg;

}


int main(int argc, char **argv)
{
  if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) {
    ros::console::notifyLoggerLevelsChanged();
  }

  ros::init(argc, argv, "updating_particles");
  ros::NodeHandle n;
  RayTracePlotter plt;

  std::random_device rd;
  std::mt19937 gen(rd());


  ROS_INFO("Running...");

  ros::Publisher probe_pub = 
    n.advertise<geometry_msgs::Pose>("/probe_point", 5);


 
  ros::Duration(2).sleep();
  
  tf::Pose probePose;
  Ray measurementRay;
  geometry_msgs::Pose probe_msg; 

  // tf::Point rStart(0.8, -0.21, 0.45);
  // tf::Point rEnd(0.8, -0.21, 0.35);


  // plt.plotRay(Ray(rStart, rEnd));


  for(int i=0; i<10; i++){
    ROS_INFO("\n------------------------------------------");
    ROS_INFO("Measurement %d", i);
    ROS_INFO("--------------------------------------------");

    getBestRandomRay(plt,  probePose, measurementRay, Scenario::MarsArm, 1000, true);
    tf::poseTFToMsg(probePose, probe_msg);


    while(!MotionStateMachine::isMotionFinished(n)){
      ROS_INFO_THROTTLE(30, "Waiting for previous movement to finish...");
      ros::spinOnce();
      ros::Duration(.1).sleep();
    }

    probe_pub.publish(probe_msg);
    ros::spinOnce();


    while(!plt.particleHandler.newParticles){
      ROS_INFO_THROTTLE(30, "Waiting for new particles...");
      ros::spinOnce();
      ros::Duration(.1).sleep();
    }
    ros::Duration(2.0).sleep();
  }


  ROS_INFO("Finished all action");

}
