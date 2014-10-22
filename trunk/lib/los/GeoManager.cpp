#include "GeoManager.h"
#include "QuadTree.h"
#include <iostream>

//======================================================================
int GeoManager::_numPolygons;
int GeoManager::_numVehiclePolygons;
int GeoManager::_numWalls;
MyPolygon GeoManager::_polygons[MAX_POLYGONS];
MyPolygon GeoManager::_vehiclePolygons[MAX_VEHICLES*10];

int GeoManager::_numVehicles;
LOSVehicle GeoManager::_vehicles[MAX_VEHICLES];
VehicleWall GeoManager::_vehicleWalls[MAX_VEHICLES];

vector<QuadTree*> GeoManager::_trees;
vector<QuadTree*> GeoManager::_vtrees;

//======================================================================
int GeoManager::numPolygons(){
  return _numPolygons;
}

//======================================================================
void GeoManager::setNumPolygons(int num){
  _numPolygons = num;
}

//======================================================================
int GeoManager::numVehiclePolygons(){
  return _numVehiclePolygons;
}

//======================================================================
void GeoManager::setNumVehiclePolygons(int num){
  _numVehiclePolygons = num;
}

//======================================================================
int GeoManager::numWalls(){
  return _numWalls;
}

//======================================================================
void GeoManager::setNumWalls(int num) {
  _numWalls = num;
}

//======================================================================
MyPolygon* GeoManager::polygons(){
  return _polygons;
}

//======================================================================
MyPolygon* GeoManager::polygon(int n) {
  return &_polygons[n];
}

//======================================================================
MyPolygon* GeoManager::vehiclePolygons(){
  return _vehiclePolygons;
}

//======================================================================
MyPolygon* GeoManager::vehiclePolygon(int n) {
  return &_vehiclePolygons[n];
}

//======================================================================
int GeoManager::numVehicles(){
  return _numVehicles;
}

//======================================================================
void GeoManager::setNumVehicles(int num){
  _numVehicles = num;
}

//======================================================================
LOSVehicle* GeoManager::vehicles(){
  return _vehicles;
}

//======================================================================
VehicleWall* GeoManager::vehicleWalls(){
  return _vehicleWalls;
}

//======================================================================
LOSVehicle* GeoManager::vehicle(int n){
  return &_vehicles[n];
}

//======================================================================
VehicleWall* GeoManager::vehicleWall(int n){
  return &_vehicleWalls[n];
}

//======================================================================
QuadTree* GeoManager::root(){
  return _trees[0];
}

//======================================================================
QuadTree* GeoManager::vroot(){
  return _vtrees[0];
}

//======================================================================
vector<QuadTree*>* GeoManager::trees(){
  return &_trees;
}

//======================================================================
vector<QuadTree*>* GeoManager::vtrees(){
  return &_vtrees;
}

//======================================================================
void GeoManager::deleteAllTrees(){
  cout << "delete all quadtrees." << endl;
  for (unsigned int i=0; i<_trees.size(); i++) {
    delete _trees[i];
  }
  _trees.clear();
  deleteVTrees();
}

//======================================================================
void GeoManager::deleteVTrees(){
  for (unsigned int i=0; i<_vtrees.size(); i++) {
    delete _vtrees[i];
  }
  _vtrees.clear();
}

//======================================================================
void GeoManager::addTree(QuadTree* tree){
  _trees.push_back(tree);
}

//======================================================================
void GeoManager::addVTree(QuadTree* vtree){
  _trees.push_back(vtree);
}
