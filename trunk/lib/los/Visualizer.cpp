#include "Visualizer.h"
#include "LOSSimulator.h"
#include "QuadTree.h"
#include "Geometry.h"
#include "GeoManager.h"
#include <autogl.h>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <sstream>
#include <string>

using namespace std;

LOSSimulator* Visualizer::_los;

double Visualizer::_xmin;
double Visualizer::_xmax;
double Visualizer::_ymin;
double Visualizer::_ymax;

int Visualizer::_polygonId = -1;

int Visualizer::_showMesh           = 0;
int Visualizer::_showVehicleMesh    = 0;
int Visualizer::_showVehicleIds     = 0;

//======================================================================
Visualizer::~Visualizer(){
}

//======================================================================
void Visualizer::setLosSimulator(LOSSimulator* los){
  _los = los;
}

//======================================================================
void Visualizer::viewRedrawCallback(){
  // 地面の描画
  AutoGL_SetColor(0.5,0.5,0.5);
  AutoGL_DrawQuadrangle(_xmin, _ymin, -5,
			_xmax, _ymin, -5,
			_xmax, _ymax, -5,
			_xmin, _ymax, -5);
  
  // 建造物壁面の描画
  int numPolygons = GeoManager::numPolygons();
  MyPolygon* polygons = GeoManager::polygons();

  for (int i=0; i<numPolygons; i++) {
    if (i==_polygonId) {
      AutoGL_SetColor(1,0.7,0.7);
    }
    else {
      AutoGL_SetColor(1,1,1);
    }
    AutoGL_DrawTriangle(polygons[i].getVector(0).getX(),
			polygons[i].getVector(0).getY(),
			polygons[i].getVector(0).getZ(),
			polygons[i].getVector(1).getX(),
			polygons[i].getVector(1).getY(),
			polygons[i].getVector(1).getZ(),
			polygons[i].getVector(2).getX(),
			polygons[i].getVector(2).getY(),
			polygons[i].getVector(2).getZ());
     
  }
  
  // 車体壁面の描画
  int numVehicles = GeoManager::numVehicles();
  const VehicleWall* vehicleWalls = GeoManager::vehicleWalls();

  for (int i=0; i<numVehicles; i++) {
    for (int j=0; j<10; j++) {
      AutoGL_SetColor(1,0,0);
      AutoGL_DrawTriangle(vehicleWalls[i].wall(j).getVector(0).getX(),
			  vehicleWalls[i].wall(j).getVector(0).getY(),
			  vehicleWalls[i].wall(j).getVector(0).getZ(),
			  vehicleWalls[i].wall(j).getVector(1).getX(),
			  vehicleWalls[i].wall(j).getVector(1).getY(),
			  vehicleWalls[i].wall(j).getVector(1).getZ(),
			  vehicleWalls[i].wall(j).getVector(2).getX(),
			  vehicleWalls[i].wall(j).getVector(2).getY(),
			  vehicleWalls[i].wall(j).getVector(2).getZ());
    }
    // idの描画
    if (_showVehicleIds){
      AutoGL_SetColor(1,1,1);
      LOSVehicle* v = GeoManager::vehicle(i);
      ostringstream ss;
      ss << v->id();
      AutoGL_DrawString(v->x(), v->y(), 4, ss.str().c_str());
    }
  }
  
  // 静的データ用四分木の描画
  double lineWidth = AutoGL_GetViewSize()*0.002;
  if (_showMesh) {
    AutoGL_SetColor(0.5,1,1);
    vector<QuadTree*>* trees = GeoManager::trees();
    for (unsigned int i=0; i<trees->size(); i++) {
      drawQuadTree((*trees)[i], lineWidth,1);
    }
  }

  // 車体用四分木の描画
  if (_showVehicleMesh) {
    AutoGL_SetColor(1,1,0.5);
    vector<QuadTree*>* vtrees = GeoManager::vtrees();
    for (unsigned int i=0; i<vtrees->size(); i++) {
      drawQuadTree((*vtrees)[i], 2*lineWidth, 0.5);
    }
  }
}

//======================================================================
void Visualizer::drawButtonCallback(){
  AutoGL_DrawView();
}

//======================================================================
void Visualizer::drawQuadTree(QuadTree* qt, double width, double height){
  double x0 = qt->xmin();
  double x1 = qt->xmax();
  double y0 = qt->ymin();
  double y1 = qt->ymax();
  AutoGL_DrawBoldLine2D(x0, y0, height, x1, y0, height, width);
  AutoGL_DrawBoldLine2D(x1, y0, height, x1, y1, height, width);
  AutoGL_DrawBoldLine2D(x1, y1, height, x0, y1, height, width);
  AutoGL_DrawBoldLine2D(x0, y1, height, x0, y0, height, width);

  // idの描画
  ostringstream ss;
  ss << qt->id();
  AutoGL_DrawString((x0+x1)*0.5, (y0+y1)*0.5, 5, ss.str().c_str());
}

//======================================================================
void Visualizer::AutoGL_DrawBoldLine2D(double x0, double y0, double z0,
				       double x1, double y1, double z1,
				       double width) {
  //方向ベクトル
  double v[3] = {x1 - x0, y1 - y0, z1 - z0};
  
  //法線ベクトル(z軸に平行な直線だと仮定している。
  double l = sqrt(v[0]*v[0] + v[1]*v[1]) ;
  double nv[3] = { -v[1] / l, v[0] / l, 0 };    

  AutoGL_DrawQuadrangle(x0 + nv[0] * width, y0 + nv[1] * width, z0,
                        x0 - nv[0] * width, y0 - nv[1] * width, z0,
                        x1 - nv[0] * width, y1 - nv[1] * width, z1,
                        x1 + nv[0] * width, y1 + nv[1] * width, z1); 
}

//======================================================================
void Visualizer::visualize(){

  _los->field(&_xmin, &_xmax, &_ymin, &_ymax);

  // AutoGLのウィンドウを用意する
  AutoGL_SetViewSize(1000);
  AutoGL_SetViewCenter((_xmin+_xmax)*0.5, (_ymin+_ymax)*0.5, 0);
  AutoGL_SetViewDirection(0,0,1);

  // View Controlパネル
  AutoGL_AddGroup(" View Control ");
  AutoGL_AddComment();
  AutoGL_SetLabel("Viewing Parameters");
  AutoGL_SetPanelInMode3D();
  AutoGL_SetMode3D(AUTOGL_MODE_3D_TRANSLATE);
  AutoGL_EnableDragInMode3D();
  AutoGL_SetDefaultCallbackInMode3D(NULL);
  AutoGL_SetViewRedrawCallback(viewRedrawCallback);

  AutoGL_AddInteger(&_polygonId, "_polygonId");
  AutoGL_SetLabel("Polygon ID");

  AutoGL_AddBoolean(&_showMesh, "_showMesh");
  AutoGL_SetLabel("Show Building Mesh");
  AutoGL_AddBoolean(&_showVehicleMesh, "_showVehicleMesh");
  AutoGL_SetLabel("Show Vehicle Mesh");
  AutoGL_AddBoolean(&_showVehicleIds, "_showVehicleIds");
  AutoGL_SetLabel("Show Vehicle ID");

  AutoGL_AddCallback(drawButtonCallback, "drawButtonCallback");
  AutoGL_SetLabel("Draw");
}
