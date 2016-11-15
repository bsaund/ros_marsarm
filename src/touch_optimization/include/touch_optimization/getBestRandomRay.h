#ifndef GET_BEST_RANDOM_RAY_H
#define GET_BEST_RANDOM_RAY_H

#include <tf/tf.h>
#include "custom_ray_trace/rayTracePlotter.h"

enum Scenario {MarsArm};

void getBestRandomRay(RayTracePlotter &plt, tf::Pose &probePose, 
			Scenario raySet, int numTrials, bool plotAll = true);



#endif
