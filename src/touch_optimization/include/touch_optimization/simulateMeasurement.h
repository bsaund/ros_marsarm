#ifndef SIMULATE_MEASUREMENT_H
#define SIMULATE_MEASUREMENT_H

#include <ros/ros.h>
#include "custom_ray_trace/rayTracer.h"


int simulateMeasurement(Ray measurementAction, RayTracer &rayt,
			 ros::ServiceClient &pfilterAdd, double noiseStdDev);

int simOnAllParts(Ray ray, std::vector<RayTracer*> &rayts, 
		   ros::ServiceClient &srv_add,
		   double noiseStdDev);



#endif
