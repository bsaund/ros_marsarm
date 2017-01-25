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
 *  Hash function for pairs. It has a lot of collisions, but who cares about performance...
 */
struct pairhash {
public:
  template <typename T, typename U>
  std::size_t operator()(const std::pair<T, U> &x) const
  {
    return ~std::hash<T>()(x.first) ^ std::hash<U>()(x.second);
  }
};




/*
 *  Relationships is a map for a specific piece "pieceA" such that
 *   Relationships["pieceB"] returns the transform distribution
 *   from B to A
 */

class PartRelationships{
 public:
  PartRelationships() {};
  PartRelationships(ros::NodeHandle &n);
  std::unordered_map<std::pair<std::string, std::string>, 
                               std::shared_ptr<TransformDistribution>,
                               pairhash> map;

  std::shared_ptr<TransformDistribution> of(std::string from, std::string to);
  bool has(std::string from, std::string to);

  bool parseRelationshipsFile(ros::NodeHandle &n);
};


/* typedef std::unordered_map<std::string, cspace> Relationships; */


bool parseJsonFile(std::string filePath, Json::Value &root);
std::shared_ptr<FixedTfDist> getFixedFromJson(Json::Value jsonTf);
std::shared_ptr<UniformTfDist> getUniformFromJson(Json::Value jsonTf);
std::shared_ptr<GaussianTfDist> getGaussianFromJson(Json::Value jsonTf);
std::shared_ptr<TransformDistribution> getTfFromJson(Json::Value jsonTf);




#endif
