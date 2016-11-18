/*
 *  Parse relationships file at the location defined by a rosparam
 *   Store only the values relevant to this namespace in the Relationships
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

typedef std::unordered_map<std::string, cspace> Relationships;

Relationships parseRelationshipsFile(ros::NodeHandle n){
  Relationships rel;
  std::string ns = ros::this_node::getNamespace();
  ns.erase(0,2);
  std::string filePath;
  if(!n.getParam("/relationshipsFile", filePath)){
    ROS_INFO("no relationships found for %s", ns.c_str());
    return rel;
  }
  std::ifstream t(filePath.c_str());
  std::string str((std::istreambuf_iterator<char>(t)),
		  std::istreambuf_iterator<char>());

  Json::Reader reader;
  Json::Value root;
  if(reader.parse(str, root)){
    /* ROS_INFO("Parsed correctly"); */
    /* ROS_INFO(ns.c_str()); */
    Json::Value::Members rootMembers = root.getMemberNames();
    for(auto mem : rootMembers){
      if(mem != ns)
	continue;
      Json::Value::Members subMembers = root[mem].getMemberNames();
      for(auto subMem : subMembers){
	cspace transform;
	Json::Value jsonTf = root[mem][subMem];
	for(int i=0; i<cdim; i++){
	  transform[i] = jsonTf[i].asDouble();
	}
	rel.insert({subMem, transform});
      }
    }
  } else {
    ROS_INFO("Parsing failed");
  }
  /* ROS_INFO("size: %d", rel.size()); */
  for(auto m : rel){
    /* ROS_INFO("%s: \[%f, %f, %f, %f, %f, %f\]", m.first.c_str(), m.second[0], */
    /* 	     m.second[1], m.second[2], m.second[3], m.second[4], m.second[5]); */
  }
  return rel;
}

#endif
