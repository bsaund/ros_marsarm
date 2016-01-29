#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <cmath>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseArray.h>
#include <random>

class ShapePlotter
{
 private:
  ros::NodeHandle n;


  tf::TransformBroadcaster br;
  ros::Publisher particle_pub;
  ros::Publisher marker_pub;

  tf::Transform transform;
  visualization_msgs::MarkerArray points;
  geometry_msgs::PoseArray particles_;

  int numParticles = 50;

 public:
  ShapePlotter();
  void updateMarkers();
  void generateTransforms();
  void plotParticles();
};



ShapePlotter::ShapePlotter()
{
  marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10);
  particle_pub = n.advertise<geometry_msgs::PoseArray>("tranform_particles", 10);
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
  for(int i=0; i<numParticles; i++){
    geometry_msgs::Pose particleTransform;
    particleTransform.position.x = randn(gen)/50;
    particleTransform.position.y = randn(gen)/50;
    particleTransform.position.z = randn(gen)/50;

    particleTransform.orientation = 
      tf::createQuaternionMsgFromRollPitchYaw(randn(gen)/50, randn(gen)/50, randn(gen)/50);
    

    // particles_.push_back(particleTransform);
    particles_.poses[i] = particleTransform;
  }
  particle_pub.publish(particles_);
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
    points.markers[i].ns = "points_and_lines";
    points.markers[i].action = visualization_msgs::Marker::ADD;

    points.markers[i].pose = particles_.poses[i];

    points.markers[i].id = i;

    points.markers[i].type = visualization_msgs::Marker::MESH_RESOURCE;
    points.markers[i].mesh_resource = "package://touch_optimization/sdf/boeing_part_binary.stl";


    // POINTS markers use x and y scale for width/height respectively
    // Boeing part is in inches, we are in meters
    points.markers[i].scale.x = .0254;
    points.markers[i].scale.y = .0254;
    points.markers[i].scale.z = .0254;


    points.markers[i].color.r = 0.5f;
    points.markers[i].color.g = 0.7f;
    points.markers[i].color.b = 0.7f;

    //alpha to make the particles transparent
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

  for(int i=0; i<particles_.poses.size(); i++){
    points.markers[i].header.stamp = ros::Time::now();
  }
  
  marker_pub.publish(points);
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
  }
  
  
  ros::spin();
}
