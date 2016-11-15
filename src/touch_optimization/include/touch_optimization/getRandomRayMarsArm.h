#ifndef GET_RANDOM_RAY_MARS_ARM_H
#define GET_RANDOM_RAY_MARS_ARM_H

#include <tf/tf.h>


void getRandomRayMarsArm(std::mt19937 &gen, tf::Pose &probePose, tf::Point &start, tf::Point &end);

#endif
