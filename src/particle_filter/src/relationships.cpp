#include "relationships.h"


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

std::shared_ptr<FixedTfDist> getFixedFromJson(Json::Value jsonTf){
  cspace cspaceTf;
  for(int i=0; i<cdim; i++){
    cspaceTf[i] = jsonTf["mean"][i].asDouble();
  }
  return std::shared_ptr<FixedTfDist>(new FixedTfDist(cspaceTf));
}


std::shared_ptr<UniformTfDist> getUniformFromJson(Json::Value jsonTf){
  cspace mean, range;
  for(int i=0; i<cdim; i++){
    mean[i] = jsonTf["mean"][i].asDouble();
    range[i] = jsonTf["range"][i].asDouble();
  }
  return std::shared_ptr<UniformTfDist>(new UniformTfDist(mean, range));
}


std::shared_ptr<GaussianTfDist> getGaussianFromJson(Json::Value jsonTf){
  cspace mean, range;
  for(int i=0; i<cdim; i++){
    mean[i] = jsonTf["mean"][i].asDouble();
    range[i] = jsonTf["range"][i].asDouble();
  }
  return std::shared_ptr<GaussianTfDist>(new GaussianTfDist(mean, range));
}

std::shared_ptr<TransformDistribution> getTfFromJson(Json::Value jsonTf){
  std::shared_ptr<TransformDistribution> tf;
  std::string type = jsonTf["type"].asString();
  if(type == "fixed"){
    tf = getFixedFromJson(jsonTf);
  } else if(type == "uniform"){
    tf = getUniformFromJson(jsonTf);
  } else if(type == "gaussian"){
    tf = getGaussianFromJson(jsonTf);
  } else{
    ROS_INFO("Unknown tf type: %s", type.c_str());
  }
  return tf;
}



std::shared_ptr<TransformDistribution> PartRelationships::of(std::string from, std::string to){
  return map[std::make_pair(from, to)];
}


/*
 *  Returns true if the relationship between from and to is defined
 */
bool PartRelationships::has(std::string from, std::string to){
  return map.count(std::make_pair(from, to)) > 0;
}


PartRelationships::PartRelationships(ros::NodeHandle &n){
  parseRelationshipsFile(n);
}


/*
 *  Reads in a json file specified by the /relationshipsFile rosparam
 *   Returns true if the initialization was successful
 */
bool PartRelationships::parseRelationshipsFile(ros::NodeHandle &n){

  std::string ns = ros::this_node::getNamespace();
  ns.erase(0,2);
  std::string filePath;
  if(!n.getParam("/relationshipsFile", filePath)){
    ROS_INFO("No relationships file parameter found for %s", ns.c_str());
    return false;
  }

  Json::Value root;
  if(!parseJsonFile(filePath, root)){
    return false;
  }

  if(!root.isMember(ns)){
    ROS_DEBUG("no relationships found for %s", ns.c_str());
  }
  
  Json::Value::Members rootMembers = root.getMemberNames();
  for(auto rootMem : rootMembers){
    Json::Value::Members subMembers = root[rootMem].getMemberNames();
    for(auto subMem : subMembers){

      Json::Value jsonTf = root[rootMem][subMem];

      std::pair<std::string, std::string> key = std::make_pair(rootMem, subMem);
      map.insert(std::make_pair(key, 
				getTfFromJson(jsonTf)));
    }
  }

  return true;
}
