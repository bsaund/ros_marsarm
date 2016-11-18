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
    cspace transform;
    Json::Value jsonTf = root[ns][subMem];
    for(int i=0; i<cdim; i++){
      transform[i] = jsonTf[i].asDouble();
    }
    RandomTransform *tf;
    tf = new FixedTransform(transform);
    /* FixedTransform tf = new FixedTransform(transform);  */
    /* std::shared_ptr<RandomTransform> tf_ptr(&tf); */
    rel.insert(std::make_pair(subMem, std::shared_ptr<RandomTransform>(tf)));
    /* rel.insert(std::make_pair(subMem, tf_ptr)); */
    /* rel.insert({subMem, transform}); */
  }

  for(auto subMem : subMembers){
    ROS_INFO("TestStart");
    ROS_INFO("Val: %f", rel[subMem]->getMean()[0]);
    ROS_INFO("TestEnd");
  }


  /* ROS_INFO("size: %d", rel.size()); */
  /* for(auto m : rel){ */
    /* ROS_INFO("%s: \[%f, %f, %f, %f, %f, %f\]", m.first.c_str(), m.second[0], */
    /* 	     m.second[1], m.second[2], m.second[3], m.second[4], m.second[5]); */
  /* } */
  return rel;
}

#endif
