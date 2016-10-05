#include "ros/ros.h"
#include "particle_filter/AddObservation.h"
#include "custom_ray_trace/rayTracePlotter.h"

particle_filter::AddObservation observation(double x, double y, double z,
					    double dirx, double diry, double dirz)
{
  particle_filter::AddObservation pfilter_obs;
  pfilter_obs.request.p.x = x;
  pfilter_obs.request.p.y = y;
  pfilter_obs.request.p.z = z;

  pfilter_obs.request.dir.x = dirx;
  pfilter_obs.request.dir.y = diry;
  pfilter_obs.request.dir.z = dirz;

  return pfilter_obs;

}



int main(int argc, char **argv){
  ros::init(argc, argv, "observe");
  ros::NodeHandle n;
  // if (argc < 8 || argc > 8){
  //   ROS_INFO("Passed %d args", argc);
  //   ROS_INFO("usage: observe: (start) x y z (dir) x y z [name]");
  //   return 1;
  // }


  // std::string name = argv[7];
  // n.setParam("localization_object", name);
  
  
  std::string srvName = "";
  if(argc >= 7)
    srvName = srvName.append("/").append(argv[6]);

  srvName.append("/particle_filter_add");




  ros::ServiceClient srv_add = 
    n.serviceClient<particle_filter::AddObservation>("particle_filter_add");

  std::vector<double> measurement;
  if(!n.getParam("measurement", measurement)){
    ROS_INFO("Failed to get param: measurement");
    return -1;
  }


  double x = measurement[0];
  double y = measurement[1];
  double z = measurement[2];
  double dx = measurement[3];
  double dy = measurement[4];
  double dz = measurement[5];

  RayTracePlotter plt;
  // tf::Point start(x,y,z);
  // tf::Point end(x+dx, y+dy, z+dz);
  // Ray ray(start, end);
  tf::Point start(0, .08, .1);
  tf::Point end(0,0.08,-.1);
  Ray ray(start, end);

  
  plt.plotRay(ray);
  double dist;
  plt.traceRay(ray, dist);
  tf::Point intersection = start + dist*(end-start).normalized();

  ros::Duration(.1).sleep();

  // particle_filter::AddObservation pfilter_obs = observation(intersection.getX(),
  // 							    intersection.getY(),
  // 							    intersection.getZ(),
  // 							    dx, dy, dz);
  particle_filter::AddObservation pfilter_obs = observation(0,0,0,
							    0,0,-1);

  if(!srv_add.call(pfilter_obs)){
    ROS_INFO("Failed to call add observation");
    ros::Duration(5).sleep();
  }
 
}
