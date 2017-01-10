#ifndef SIMULATE_MEASUREMENT_H
#define SIMULATE_MEASUREMENT_H

#include <ros/ros.h>
#include "custom_ray_trace/rayTracer.h"
#include "particle_filter/relationships.h"

int simulateMeasurement(Ray measurementAction, RayTracer &rayt,
			 ros::ServiceClient &pfilterAdd, double noiseStdDev);

int simOnAllParts(Ray ray, std::vector<RayTracer*> &rayts, 
		   ros::ServiceClient &srv_add,
		   double noiseStdDev);

std::vector<RayTracer*> getAllRayTracers();

bool getIntersectingRayTracer(Ray ray, std::vector<RayTracer*> &rayts, RayTracer* &hitPart);

double getIG(Ray ray, std::vector<RayTracer*> rayts, Relationships rel, 
	     double radialErr, double depthErr);


#endif
