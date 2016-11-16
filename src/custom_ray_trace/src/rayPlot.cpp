#include "ros/ros.h"
#include "rayTracePlotter.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "ray_trace_test");
  if (argc < 7 || argc > 9){
    ROS_INFO("usage: Plots vector with IG: (start) x y z (end)x y z (radial err)e (bin size)s");
    return 1;
  }
  double radErr = (argc >= 8) ? atof(argv[7]) : 0.01;
  double binSize = (argc >= 9) ? atof(argv[8]) : 0.01;

  ROS_INFO("Showing IG for ray with radial error %f and bin size %f", radErr, binSize);

  RayTracePlotter plt;


  tf::Point start = tf::Point(atof(argv[1]),
			      atof(argv[2]),
			      atof(argv[3]));

  tf::Point end = tf::Point(atof(argv[4]),
			    atof(argv[5]),
			    atof(argv[6]));
  double dist;

  Ray ray(start, end);


  // plt.plotRay(ray);
  plt.plotCylinder(ray, .01);
  plt.plotIntersections(ray);
  double IG = plt.getIG(ray, radErr, binSize);
  // plt.labelRay(ray, IG);
  ROS_INFO("IG: %f", IG);



  return 0;
}
  












