#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include "simpleShape.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>
#include <random>

class ShapePlotter
{
 private:
  ros::NodeHandle n;
  ros::Publisher marker_pub;

  tf::TransformBroadcaster br;
  tf::Transform transform;
  visualization_msgs::MarkerArray points;

 public:
  ShapePlotter();
  void generateParticles();
  void plotParticles();
};



ShapePlotter::ShapePlotter()
{
  marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
}


void ShapePlotter::generateParticles()
{
  int numMarkers = 50;

  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> randn(0, 1);


 


  points.markers.resize(numMarkers);
  for(int i=0; i<numMarkers; i++){
    points.markers[i].header.frame_id = "particle_frame";
    points.markers[i].header.stamp = ros::Time::now();
    points.markers[i].ns = "points_and_lines";
    points.markers[i].action = visualization_msgs::Marker::ADD;

    points.markers[i].pose.position.x = 0.0 + randn(gen)/50;
    points.markers[i].pose.position.y = 0.0 + randn(gen)/50;
    points.markers[i].pose.position.z = 0.0 + randn(gen)/50;

    // points.markers[i].pose.orientation.w = 1.0;
    points.markers[i].pose.orientation = 
      tf::createQuaternionMsgFromRollPitchYaw(randn(gen)/50, randn(gen)/50, randn(gen)/50);

    points.markers[i].id = i;

    points.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
    points.markers[i].mesh_resource = "package://touch_optimization/sdf/boeing_part_binary.stl";


    // POINTS markers use x and y scale for width/height respectively
    points.markers[i].scale.x = .0254;
    points.markers[i].scale.y = .0254;
    points.markers[i].scale.z = .0254;


    // %Tag(COLOR)%
    // Points are green
    points.markers[i].color.r = 0.5f;
    points.markers[i].color.g = 0.7f;
    points.markers[i].color.b = 0.7f;

    points.markers[i].color.a = 0.1;

  }
}

void ShapePlotter::plotParticles(){
  geometry_msgs::TransformStamped trans;
  tf::Transform unityTransform;
  tf::Transform particleTransform;
  particleTransform.setOrigin(tf::Vector3(1,2,3));
  tf::StampedTransform tfstmp(particleTransform, ros::Time::now(),"my_frame", "particle_frame");
  tf::transformStampedTFToMsg(tfstmp, trans);
  
  br.sendTransform(trans);
  tfstmp =   tf::StampedTransform(unityTransform, ros::Time::now(),"base_plate", "my_frame");
  tf::transformStampedTFToMsg(tfstmp, trans);

  br.sendTransform(trans);
  int numMarkers = 50;


  for(int i=0; i<numMarkers; i++){

    points.markers[i].header.stamp = ros::Time::now();

  }

 
 
  marker_pub.publish(points);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "plotter");

  ShapePlotter plt;

  // Cube c;

  // tf::Transform transform;
  ros::Duration waitForRViz(1.0);
  plt.generateParticles();


  while (ros::ok()) {
    waitForRViz.sleep();
    plt.plotParticles();

    // double t = ros::Time::now().toSec();
    // transform.setOrigin( tf::Vector3(2.0*sin(t),
    // 				     2.0*cos(t),
    // 				     0.0) );
    // tf::Quaternion q;
    // q.setRPY(t,0,0);
    // transform.setRotation(q);

    // plt.plotShape(c, tf::StampedTransform(transform, ros::Time::now(), "my_frame", "newFrame1"));
    // ros::spinOnce();
  }
  
  
  ros::spin();
}
