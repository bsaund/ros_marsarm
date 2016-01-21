#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <cmath>
#include "plotShape.h"
#include "simpleShape.h"
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Transform.h>




ShapePlotter::ShapePlotter()
{
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);
}


void ShapePlotter::plotShape(const Cube shape, tf::StampedTransform trans)
{


  geometry_msgs::TransformStamped gtr;
  tf::transformStampedTFToMsg(trans, gtr);

  plotShape(shape, gtr);
}

void ShapePlotter::plotShape(const Cube shape, const geometry_msgs::TransformStamped trans)
{

  br.sendTransform(trans);


  // %Tag(MARKER_INIT)%
  visualization_msgs::Marker points, line_strip, line_list;
  points.header.frame_id = line_strip.header.frame_id = line_list.header.frame_id = "newFrame1";
  points.header.stamp = line_strip.header.stamp = line_list.header.stamp = ros::Time::now();
  points.ns = line_strip.ns = line_list.ns = "points_and_lines";
  points.action = line_strip.action = line_list.action = visualization_msgs::Marker::ADD;
  points.pose.orientation.w = line_strip.pose.orientation.w = line_list.pose.orientation.w = 1.0;
  // %EndTag(MARKER_INIT)%

  // %Tag(ID)%
  points.id = 0;
  line_strip.id = 1;
  line_list.id = 2;
  // %EndTag(ID)%

  // %Tag(TYPE)%
  points.type = visualization_msgs::Marker::POINTS;
  line_strip.type = visualization_msgs::Marker::LINE_STRIP;
  line_list.type = visualization_msgs::Marker::LINE_LIST;
  // %EndTag(TYPE)%

  // %Tag(SCALE)%
  // POINTS markers use x and y scale for width/height respectively
  points.scale.x = 0.2;
  points.scale.y = 0.2;

  // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
  line_strip.scale.x = 0.1;
  line_list.scale.x = 0.1;
  // %EndTag(SCALE)%

  // %Tag(COLOR)%
  // Points are green
  points.color.g = 1.0f;
  points.color.a = 1.0;

  // Line strip is blue
  line_strip.color.b = 1.0;
  line_strip.color.a = 1.0;

  // Line list is red
  line_list.color.r = 1.0;
  line_list.color.a = 1.0;
  // %EndTag(COLOR)%


  for(int i=0; i < shape.numEdges(); i++){
    geometry_msgs::Point p;
    Edge edge = shape.getEdge(i);
    // p = edge.point1;
    tf::pointTFToMsg(edge.point1, p);
    points.points.push_back(p);
    line_list.points.push_back(p);

    tf::pointTFToMsg(edge.point2, p);
    points.points.push_back(p);
    line_list.points.push_back(p);
      
  }

  marker_pub.publish(points);
  // marker_pub.publish(line_strip);
  marker_pub.publish(line_list);




}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "plotter");

  ShapePlotter plt;
  Cube c;

  tf::Transform transform;

  while (ros::ok()) {
    double t = ros::Time::now().toSec();
    transform.setOrigin( tf::Vector3(2.0*sin(t),
    				     2.0*cos(t),
    				     0.0) );
    tf::Quaternion q;
    q.setRPY(t,0,0);
    transform.setRotation(q);

    plt.plotShape(c, tf::StampedTransform(transform, ros::Time::now(), "my_frame", "newFrame1"));
  }
  
  
  ros::spin();
}
