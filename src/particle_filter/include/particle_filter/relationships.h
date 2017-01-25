/*
 *  Parse relationships file at the location defined by a rosparam
 *   Store only the relationships relevant to this namespace in the Relationships
 */

#ifndef RELATIONSHIPS_H
#define RELATIONSHIPS_H

#include "json/jsonInternalLib.h"
#include <ros/ros.h>
#include <string>
#include <fstream>
#include <streambuf>
#include "PfDefinitions.h"
#include <unordered_map>
#include "transformDistribution.h"


/*
 *  Relationships is a map for a specific piece "pieceA" such that
 *   Relationships["pieceB"] returns the transform distribution
 *   from B to A
 */
typedef std::unordered_map<std::string, std::shared_ptr<TransformDistribution>> Relationships;
/* typedef std::unordered_map<std::string, cspace> Relationships; */


bool parseJsonFile(std::string filePath, Json::Value &root);
std::shared_ptr<FixedTfDist> getFixedFromJson(Json::Value jsonTf);
std::shared_ptr<UniformTfDist> getUniformFromJson(Json::Value jsonTf);
std::shared_ptr<GaussianTfDist> getGaussianFromJson(Json::Value jsonTf);
std::shared_ptr<TransformDistribution> getTfFromJson(Json::Value jsonTf);
Relationships parseRelationshipsFile(ros::NodeHandle n);



#endif
