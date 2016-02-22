#include <ros/ros.h>
#include "particle_filter/PFilterInit.h"
#include "geometry_msgs/Point.h"
#include <tf/transform_broadcaster.h>
#include "gazebo_ray_trace/plotRayUtils.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "updating_particles");
  ros::NodeHandle n;
  PlotRayUtils plt;
  ROS_INFO("Running...");

  ros::Publisher pub_init = 
    n.advertise<particle_filter::PFilterInit>("/particle_filter_init", 5);
  ros::Publisher pub_add = 
    n.advertise<geometry_msgs::Point>("/particle_filter_add", 5);

  particle_filter::PFilterInit init_points;
  tf::pointTFToMsg(tf::Point(0,0,0),  init_points.p1);
  tf::pointTFToMsg(tf::Point(0,1,-1), init_points.p2);
  tf::pointTFToMsg(tf::Point(1,-1,0), init_points.p3);
 
  ros::Duration(2).sleep();
  pub_init.publish(init_points);
 
  geometry_msgs::Point obs;
 
  for(int i=0; i<10; i++){
    ros::Duration(1).sleep();
    obs.x=0.1; obs.y=0.1; obs.z=0;
    pub_add.publish(obs);

    ros::Duration(4).sleep();
    plt.plotCylinder(tf::Point(1,1,1), tf::Point(-1,-1,-1), 0.01, 0.002, true);
    
  }

  ROS_INFO("Finished all action");



 
}
