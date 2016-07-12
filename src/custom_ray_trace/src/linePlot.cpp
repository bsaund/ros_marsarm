#include "ros/ros.h"
#include "rayTracer.h"
#include "plotRayUtils.h"

int main(int argc, char **argv){
  ros::init(argc, argv, "ray_trace_test");
  if (argc < 11){
    ROS_INFO("usage: (ray start) x y z, (ray end) x y z, (line dir)x y z, (num rays)n, radErr, binSize");
    return 1;
  }

  RayTracer rayTracer;
  PlotRayUtils plt;


  tf::Point start = tf::Point(atof(argv[1]),
			      atof(argv[2]),
			      atof(argv[3]));

  tf::Point end = tf::Point(atof(argv[4]),
			    atof(argv[5]),
			    atof(argv[6]));

  tf::Vector3 dir = tf::Vector3(atof(argv[7]),
				atof(argv[8]),
				atof(argv[9]));

  int n = atoi(argv[10]);

  double radErr = (argc >= 12) ? atof(argv[11]) : 0.01;
  double binSize = (argc >= 13) ? atof(argv[12]) : 0.9;

  ROS_INFO("Showing IG for ray with radial error %f and bin size %f", radErr, binSize);


  double dist;


  // Ray ray(start, end);
  std::vector<double> distsQuick;
  std::vector<double> dists;

  Ray ray_base(start, end);

  std::vector<Ray> rays;
  for(int i=0; i<n; i++){
    // double height = 0.3 + (double)i/(5*input1);
    rays.push_back(Ray(start + dir*i/n, end + dir*i/n));
  }

  for(Ray ray:rays){
    plt.plotRay(ray, false);
    plt.labelRay(ray, rayTracer.getIG(ray, radErr, binSize));
  }

  double combIG = rayTracer.getIG(rays, radErr, binSize);
  std::stringstream s;
  s << "Combined IG: " << combIG;
  plt.label(tf::Point(1,0,0), 10, s.str());

  // ROS_INFO("IG of base ray: %f", rayTracer.getIG(ray_base, 0.01, 0.01));
  // ROS_INFO("IG of input ray: %f", rayTracer.getIG(ray, 0.01, 0.01));
  ROS_INFO("IG of both: %f", combIG);

  return 0;
}
  












