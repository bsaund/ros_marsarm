/*
 *  Parse relationships file at the location defined by a rosparam
 *   Store only the relationships relevant to this namespace in the Relationships
 */

#ifndef RELATIONSHIPS_H
#define RELATIONSHIPS_H

#include "json/json.h"
#include <ros/ros.h>
#include <string>
#include <fstream>
#include <streambuf>
#include "PfDefinitions.h"
#include <unordered_map>
#include "randomTransform.h"


typedef std::unordered_map<std::string, std::shared_ptr<RandomTransform>> Relationships;
/* typedef std::unordered_map<std::string, cspace> Relationships; */

/*
 *  Parses a Json file given the filepath
 */
bool parseJsonFile(std::string filePath, Json::Value &root){
  std::ifstream t(filePath.c_str());
  std::string str((std::istreambuf_iterator<char>(t)),
		  std::istreambuf_iterator<char>());
  Json::Reader reader;  
  if(!reader.parse(str, root)){
    ROS_INFO("Parsing Json File '%s' failed", filePath.c_str());
    return false;
  }
}

std::shared_ptr<FixedTransform> getFixedFromJson(Json::Value jsonTf){
  cspace cspaceTf;
  for(int i=0; i<cdim; i++){
    cspaceTf[i] = jsonTf["mean"][i].asDouble();
  }
  return std::shared_ptr<FixedTransform>(new FixedTransform(cspaceTf));
}

std::shared_ptr<RandomTransform> getTfFromJson(Json::Value jsonTf){
  std::shared_ptr<RandomTransform> tf;
  std::string type = jsonTf["type"].asString();
  if(type == "fixed"){
      tf = getFixedFromJson(jsonTf);
  } else if(type == "normal"){
  } else{
    ROS_INFO("Unknown tf type: %s", type.c_str());
  }
  return tf;
}

Relationships parseRelationshipsFile(ros::NodeHandle n){
  Relationships rel;
  std::string ns = ros::this_node::getNamespace();
  ns.erase(0,2);
  std::string filePath;
  if(!n.getParam("/relationshipsFile", filePath)){
    ROS_INFO("No relationships file parameter found for %s", ns.c_str());
    return rel;
  }

  Json::Value root;
  if(!parseJsonFile(filePath, root)){
    return rel;
  }

  if(!root.isMember(ns)){
    ROS_DEBUG("no relationships found for %s", ns.c_str());
    return rel;
  }

  Json::Value::Members subMembers = root[ns].getMemberNames();
  for(auto subMem : subMembers){

    Json::Value jsonTf = root[ns][subMem];

    rel.insert(std::make_pair(subMem, 
			      getTfFromJson(jsonTf)));
  }

  return rel;
}

#endif
