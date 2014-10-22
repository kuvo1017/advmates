#ifndef __GEO_MANAGER_H__
#define __GEO_MANAGER_H__

#include "Geometry.h"
#include "LOSConf.h"
#include <vector>

using namespace std;

class QuadTree;

class GeoManager{
 public:
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // 有効なポリゴン総数を返す
  static int numPolygons();
  // 有効なポリゴン総数をセットする
  static void setNumPolygons(int num);
  // 車体ポリゴン総数を返す
  static int numVehiclePolygons();
  // 車体ポリゴン総数をセットする
  static void setNumVehiclePolygons(int num);
  // 壁総数を返す
  static int numWalls();
  // 壁総数をセットする
  static void setNumWalls(int num);
  // ポリゴンの集合を返す
  static MyPolygon* polygons();
  // n番目のポリゴンを返す
  static MyPolygon* polygon(int n);
  // 車体用ポリゴンを返す
  static MyPolygon* vehiclePolygons();
  // n番目の車体用ポリゴンを返す
  static MyPolygon* vehiclePolygon(int n);

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // 有効な車体総数を返す
  static int numVehicles();
  // 有効な車体総数をセットする
  static void setNumVehicles(int num);
  // 車体の集合を返す
  static LOSVehicle* vehicles();
  // 車体面の集合を返す
  static VehicleWall* vehicleWalls();
  // n番目の車体を返す
  static LOSVehicle* vehicle(int n);
  // n番目の車体面を返す
  static VehicleWall* vehicleWall(int n);

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // 静的データ用四分木ルートを返す
  static QuadTree* root();
  // 車体用四分木のルートを返す
  static QuadTree* vroot();
  // 静的データ用四分木を返す
  static vector<QuadTree*>* trees();
  // 車体用四分木を返す
  static vector<QuadTree*>* vtrees();
  // 全ての四分木を消去する
  static void deleteAllTrees();
  // 車体用四分木を消去する
  static void deleteVTrees();
  // 静的データ用四分木を追加する
  static void addTree(QuadTree* tree);
  // 車体用四分木を追加する
  static void addVTree(QuadTree* vtree);

 private:
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // 有効なポリゴン総数
  static int _numPolygons;
  // 車体ポリゴン総数
  static int _numVehiclePolygons;
  // 壁総数
  static int _numWalls;
  // ポリゴンの集合
  static MyPolygon _polygons[MAX_POLYGONS];
  // 車体用ポリゴンの集合
  static MyPolygon _vehiclePolygons[MAX_VEHICLES*10];

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // 有効な車体総数
  static int _numVehicles;
  // 車体の集合
  static LOSVehicle _vehicles[MAX_VEHICLES];
  // 車体面
  static VehicleWall _vehicleWalls[MAX_VEHICLES];

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // 四分木の集合(メインコンテナ)
  // ここでは木構造になっていない
  static vector<QuadTree*> _trees;

  // 車体を格納する四分木（メインコンテナ）
  static vector<QuadTree*> _vtrees;
};

#endif //__GEO_MANAGER_H__
