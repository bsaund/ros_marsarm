/*  
 *  Generates particles in SE(3) for the configuration of an object
 *  Plots these particles by sending markers to RViz
 *  Published the particles so other ros nodes have access
 */

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <random>
#include <iostream>

class ShapePlotter
{
 private:
  ros::NodeHandle n;


  tf::TransformBroadcaster br;
  ros::Publisher particle_pub;
  ros::Publisher marker_pub;

  ros::Subscriber sub;

  tf::Transform transform;
  visualization_msgs::MarkerArray points;
  geometry_msgs::PoseArray particles_;

  int numParticles = 50;

  void externalParticleUpdate(geometry_msgs::PoseArray p);

 public:
  ShapePlotter();
  void updateMarkers();
  void generateTransforms();
  void plotParticles();
};



ShapePlotter::ShapePlotter()
{
  marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
  particle_pub = n.advertise<geometry_msgs::PoseArray>("/transform_particles", 10);
  sub = n.subscribe("/particles_from_filter", 1, &ShapePlotter::externalParticleUpdate, this);
}

void ShapePlotter::externalParticleUpdate(geometry_msgs::PoseArray p)
{
  ROS_INFO("Particles Updated");
  particles_ = p;
  updateMarkers();
  plotParticles();
}

/** 
 *  Generates the particles in random configureation
 */
void ShapePlotter::generateTransforms()
{
  std::random_device rd;
  std::mt19937 gen(rd());
  std::normal_distribution<> randn(0, 1);
  
  particles_.poses.resize(numParticles);

  for(int i=0; i<particles_.poses.size(); i++){
    geometry_msgs::Pose particleTransform;
    // particleTransform.position.x = randn(gen)/50;
    // particleTransform.position.y = randn(gen)/50;
    // particleTransform.position.z = randn(gen)/50;
    particleTransform.position.x = 0;
    particleTransform.position.y = 0;
    particleTransform.position.z = 0;

    particleTransform.orientation = 
      tf::createQuaternionMsgFromRollPitchYaw(randn(gen)/20, randn(gen)/20, randn(gen)/20);
      // tf::createQuaternionMsgFromRollPitchYaw(0, randn(gen)/20, 0);
    

    // particles_.push_back(particleTransform);
    particles_.poses[i] = particleTransform;
  }
  // particle_pub.publish(particles_);
  // ROS_INFO("About to call particle size");
  // ROS_INFO("Publishing particles of size %n", particles_.poses.size());
  // ROS_INFO("Particle size correct");
}


/** 
 *  Update markers according to their particle transforms
 */
void ShapePlotter::updateMarkers()
{
  points.markers.resize(particles_.poses.size());
  for(int i=0; i<particles_.poses.size(); i++){
    points.markers[i].header.frame_id = "particle_frame";
    points.markers[i].header.stamp = ros::Time::now();
    points.markers[i].ns = "particles";
    points.markers[i].action = visualization_msgs::Marker::ADD;

    points.markers[i].pose = particles_.poses[i];

    points.markers[i].id = i;

    points.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;

    //Change this in two locations: Here and gazebo
    // points.markers[i].mesh_resource = "package://touch_optimization/sdf/boeing_part_binary.stl";
    // points.markers[i].mesh_resource = "package://touch_optimization/sdf/flat_plate.stl";
    std::string s;
   
    if( n.getParam("/localization_object_cad", s)){
      // ROS_INFO("Localization object is:");
      // ROS_INFO(s.c_str());
    } else{
      ROS_INFO("Failed to get param");
    }
    // std::cout << "LOCALIZATION OBJECT: " << s.c_str() << std::endl;
    points.markers[i].mesh_resource = s;
    
    // POINTS markers use x and y scale for width/height respectively
    // Boeing part is in inches, we are in meters
    points.markers[i].scale.x = .0254;
    points.markers[i].scale.y = .0254;
    points.markers[i].scale.z = .0254;

    points.markers[i].color.r = 0.74f;
    points.markers[i].color.g = 0.78f;
    points.markers[i].color.b = 0.8f;

    //alpha to make the particles transparent
    points.markers[i].color.a = 0.07;

  }
}


/**
 * Publishes both the marker points and particle transforms
 */
void ShapePlotter::plotParticles(){
  geometry_msgs::TransformStamped trans;
  tf::Transform unityTransform;
  unityTransform.setOrigin(tf::Vector3(0,0,0));
  tf::Quaternion q;
  q.setRPY(0,0,0);
  unityTransform.setRotation(q);

  tf::Transform particleTransform;
  // particleTransform.setOrigin(tf::Vector3(1,1,1));
  particleTransform.setOrigin(tf::Vector3(0,0,0));
  q.setRPY(-.7, 1.5, 0);
  particleTransform.setRotation(q);

  tf::StampedTransform tfstmp(particleTransform, ros::Time::now(),"my_frame", "particle_frame");
  tf::transformStampedTFToMsg(tfstmp, trans);
    br.sendTransform(trans);

  tfstmp = tf::StampedTransform(unityTransform, ros::Time::now(),"base_plate", "my_frame");
    tf::transformStampedTFToMsg(tfstmp, trans);
  br.sendTransform(trans);

  for(int i=0; i<particles_.poses.size(); i++){
    points.markers[i].header.stamp = ros::Time::now();
  }
  
  marker_pub.publish(points);

  particle_pub.publish(particles_);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "plotter");

  ShapePlotter plt;

  ros::Duration waitForRViz(1.0);

  plt.generateTransforms();
  plt.updateMarkers();


  while (ros::ok()) {
    waitForRViz.sleep();
    plt.plotParticles();
    // ROS_INFO("spinning");
    ros::spinOnce();
  }
  
  
  ros::spin();
}
