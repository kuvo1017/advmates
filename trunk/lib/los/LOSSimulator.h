#ifndef __LOS_SIMULATOR_H__
#define __LOS_SIMULATOR_H__

#include <string>
#include <vector>
#include <map>
#include "Geometry.h"
#include "LOSConf.h"
#include "shapefil.h"

using namespace std;

class QuadTree;
class Vehicle;

class LOSSimulator{
 public:
  LOSSimulator();
  ~LOSSimulator();

  // ファイル読み込みと建造物ポリゴンデータを作成する
  void init();

  // 全車両間の可視判定を行う
  /* 結果は配列に格納する */
  void calcVehicleLOS(vector<Vehicle*>* vehicles);

  // 地図領域を返す
  void field(double *xmin, double *xmax,
	     double *ymin, double *ymax) const;

 private:
  // ファイル読み込み関数
  void _readFiles(string shpFileName, string dbfFileName);
  // SHPファイルハンドラ
  SHPHandle _hSHP;
  // DBFファイルハンドラ
  DBFHandle _hDBF;

  // ポリゴンの作成
  void _makePolygon(int entityId, int partId, SHPObject* shape);

  // 線分の傾きとy切片を算出
  void _calcStraightLine(LOSVehicle* vt, LOSVehicle* vr, double* a, double* b);

  // 境界
  double _adfMaxBounds[4];
  double _adfMinBounds[4];

  // ポリゴン総数
  int _numPolygons;
  // 壁総数
  int _numWalls;
  // 車体総数
  int _numVehicles;
 
  // MATESのID（int に直したもの）とLOS用のIDの関連付け
  map<int,int> _idMap;

  // 結果を格納する配列
  int _los[MAX_VEHICLES*MAX_VEHICLES];
};
#endif //__LOS_SIMULATOR_H__
