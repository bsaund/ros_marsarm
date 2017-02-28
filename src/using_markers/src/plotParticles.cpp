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
  ros::Publisher marker_true_pub;

  ros::Subscriber sub;

  tf::Transform transform;
  visualization_msgs::MarkerArray points;
  visualization_msgs::Marker part;
  geometry_msgs::PoseArray particles_;
  
  std::string cadPath;  
  std::string name;  

  int numParticles = 1;

  void externalParticleUpdate(geometry_msgs::PoseArray p);

 public:
  ShapePlotter();
  void updateMarkers();
  void updateTrueMarker();
  void generateTransforms();
  void plotParticles();
  void plotTruePart();
};



ShapePlotter::ShapePlotter()
{
  marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
  marker_true_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  particle_pub = n.advertise<geometry_msgs::PoseArray>("/transform_particles", 10);
  sub = n.subscribe("particles_from_filter", 1, &ShapePlotter::externalParticleUpdate, this);

  if(!n.getParam("localization_object_cad", cadPath)){
    ROS_INFO("Failed to get param: localization_object_cad");
  }

  if(!n.getParam("localization_object", name)){
    ROS_INFO("Failed to get param: localization_object");
  }

}

/*
 * Callback for a ros listener to allow external ros nodes to
 *  update particles
 */
void ShapePlotter::externalParticleUpdate(geometry_msgs::PoseArray p)
{
  ROS_INFO("Plot of %s updated due to new particles", name.c_str());
  particles_ = p;
  updateMarkers();
  plotParticles();
  particle_pub.publish(particles_);
}

/** 
 *  Generates the particles in random configureation
 */
void ShapePlotter::generateTransforms()
{
  std::random_device rd;
  std::normal_distribution<> randn(0, 1);
  
  particles_.poses.resize(numParticles);
  std::vector<double> uncertainties;
  if(!n.getParam("/initial_uncertainties", uncertainties)){
    ROS_INFO("Failed to get param initial uncertainties");
    uncertainties.resize(6);
  }


  for(int i=0; i<particles_.poses.size(); i++){
    geometry_msgs::Pose particleTransform;
    // particleTransform.position.x = randn(gen)/50;
    // particleTransform.position.y = randn(gen)/50;
    // particleTransform.position.z = randn(gen)/50;s


    // particleTransform.position.x = randn(gen)*uncertainties[0];
    // particleTransform.position.y = randn(gen)*uncertainties[1];
    // particleTransform.position.z = randn(gen)*uncertainties[2];

    // particleTransform.orientation = 
    //   tf::createQuaternionMsgFromRollPitchYaw(randn(gen)*uncertainties[3], 
    // 					      randn(gen)*uncertainties[4], 
    // 					      randn(gen)*uncertainties[5]);
    particleTransform.position.x = 0;
    particleTransform.position.y = 0;
    particleTransform.position.z = 0;

    particleTransform.orientation = 
      tf::createQuaternionMsgFromRollPitchYaw(0,0,0);
					      
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
    points.markers[i].header.frame_id = name;
    points.markers[i].header.stamp = ros::Time::now();
    points.markers[i].ns = "particles";
    points.markers[i].action = visualization_msgs::Marker::ADD;

    points.markers[i].pose = particles_.poses[i];

    points.markers[i].id = i;

    points.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;

    //Change this in two locations: Here and gazebo
    // points.markers[i].mesh_resource = "package://touch_optimization/sdf/boeing_part_binary.stl";
    // points.markers[i].mesh_resource = "package://touch_optimization/sdf/flat_plate.stl";

   

    points.markers[i].mesh_resource = cadPath;
    
    // POINTS markers use x and y scale for width/height respectively
    // Boeing part is in inches, we are in meters
    points.markers[i].scale.x = .0254;
    points.markers[i].scale.y = .0254;
    points.markers[i].scale.z = .0254;

    // points.markers[i].color.r = 0.74f;
    // points.markers[i].color.g = 0.78f;
    // points.markers[i].color.b = 0.8f;

    // points.markers[i].color.r = 180.0f/255;  //236
    // points.markers[i].color.g = 145.0f/255;  //255
    // points.markers[i].color.b = 120.0f/255;   //106

    std::vector<double> color;
    if(!n.getParam("color", color)){
      ROS_INFO("Failed to get param: color");
	color.resize(4);
    }
    if(color.size() != 4){
      ROS_INFO_THROTTLE(60,"Color Vector wrong size: [r g b alpha]");
      color.resize(4);
      color[3] = 1;
    }

    points.markers[i].color.r = color[0];
    points.markers[i].color.g = color[1];
    points.markers[i].color.b = color[2];


    //alpha to make the particles transparent
    // points.markers[i].color.a = 0.07;
    points.markers[i].color.a = color[3];

  }
}


void ShapePlotter::updateTrueMarker()
{
    part.header.frame_id = "true_frame";
    part.header.stamp = ros::Time(0);
    part.ns = "true_part";
    part.action = visualization_msgs::Marker::ADD;

    //tf::Pose pose;
    part.pose.position.x = 0;
    part.pose.position.y = 0;
    part.pose.position.z = 0;
    part.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

    part.id = 1;

    part.type = visualization_msgs::Marker::MESH_RESOURCE;

    //Change this in two locations: Here and gazebo
    // part.mesh_resource = "package://touch_optimization/sdf/boeing_part_binary.stl";
    // part.mesh_resource = "package://touch_optimization/sdf/flat_plate.stl";
   
    part.mesh_resource = cadPath;
    
    // POINTS markers use x and y scale for width/height respectively
    // Boeing part is in inches, we are in meters
    part.scale.x = .0254;
    part.scale.y = .0254;
    part.scale.z = .0254;

    part.color.r = 1.0f;
    part.color.g = 0.0f;
    part.color.b = 0.0f;

    //alpha to make the particles transparent
    part.color.a = 1.0;

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

  // tf::Transform particleTransform;
  // particleTransform.setOrigin(tf::Vector3(1,1,1));

  // std::vector<double> pFrame;
  // if(!n.getParam("particle_frame", pFrame)){
  //   ROS_INFO("Failed to get param: particle_frame");
  //   pFrame.resize(6);
  // }


  // particleTransform.setOrigin(tf::Vector3(pFrame[0],pFrame[1],pFrame[2]));
  // // q.setRPY(-.7, 1.5, 0);
  // q.setRPY(pFrame[3],pFrame[4], pFrame[5]);
  // particleTransform.setRotation(q);
    
  // tf::StampedTransform tfstmp(particleTransform, ros::Time::now(),"my_frame", name);
  tf::StampedTransform tfstmp(unityTransform, ros::Time::now(),"my_frame", name);
  tf::transformStampedTFToMsg(tfstmp, trans);
    br.sendTransform(trans);

  tfstmp = tf::StampedTransform(unityTransform, ros::Time::now(),"base_plate", "my_frame");
  tf::transformStampedTFToMsg(tfstmp, trans);
  br.sendTransform(trans);

  std::vector<double> trueFrame;
  if(n.getParam("true_frame", trueFrame)){
    tf::Transform trueTransform;
    trueFrame.resize(6);
    trueTransform.setOrigin(tf::Vector3(trueFrame[0],trueFrame[1],trueFrame[2]));
    q.setRPY(trueFrame[3],trueFrame[4], trueFrame[5]);
    trueTransform.setRotation(q);
    tfstmp = tf::StampedTransform(trueTransform, ros::Time::now(),"my_frame", "true_frame");
    tf::transformStampedTFToMsg(tfstmp, trans);
    br.sendTransform(trans);
    plotTruePart();
  }



  for(int i=0; i<particles_.poses.size(); i++){
    points.markers[i].header.stamp = ros::Time::now();
  }
  
  marker_pub.publish(points);

  particle_pub.publish(particles_);
}

void::ShapePlotter::plotTruePart()
{
  marker_true_pub.publish(part);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "plotter");

  ShapePlotter plt;

  ros::Duration waitForRViz(.01);

  // plt.generateTransforms();
  plt.updateMarkers();
  plt.updateTrueMarker();

  ros::Time prevT = ros::Time::now();

  while (ros::ok()) {
    
    waitForRViz.sleep();
    if((ros::Time::now() - prevT).toSec() > 1){
      prevT = ros::Time::now();
      plt.plotParticles();
    }
    // ROS_INFO("spinning");
    ros::spinOnce();
  }
  
  
  ros::spin();
}
