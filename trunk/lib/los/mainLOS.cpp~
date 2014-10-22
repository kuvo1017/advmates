#include <iostream>
#include <vector>
#include <memory>
#include <autogl.h>
#include "LOSSimulator.h"
#include "Visualizer.h"
#include "Simulator.h"
#include "ObjManager.h"

using namespace std;

//######################################################################
class AppLOS{
private:
  LOSSimulator* _los;
  Visualizer*   _vis;
  Simulator*    _sim;
public:
  AppLOS(){};
  ~AppLOS(){
    if (_los) delete _los;
    if (_vis) delete _vis;
    if (_sim) delete _sim;
  };
  void init();
  int run();
};
//======================================================================
void AppLOS::init(){
  if (_los) delete _los;
  _los = new LOSSimulator();
  if (_vis) delete _vis;
  _vis = new Visualizer();
  if (_sim) delete _sim;
  _sim = new Simulator();

  // ファイルを読み込み建造物ポリゴンデータを生成する
  _los->init();
  
  // ダミーの車両データを作成し、取得する
  double xmin, xmax, ymin, ymax;
  _los->field(&xmin, &xmax, &ymin, &ymax);
  _sim->startSampleScenario((xmin+xmax)*0.5-1000,
			    (xmin+xmax)*0.5+1000,
			    (ymin+ymax)*0.5-1000,
			    (ymin+ymax)*0.5+1000,
			    1000, 1000, 20, 50);
  vector<Vehicle*>* vehicles = ObjManager::vehicles();

  // 車両間の可視判定
  _los->calcVehicleLOS(vehicles);

  // 可視化オブジェクトに渡す
  _vis->setLosSimulator(_los);
}
//======================================================================
int AppLOS::run(){
  if (_vis) {
    // 入力されたポリゴンデータの表示
    _vis->visualize();
    return 0;
  }
  else {
    return 1;
  }
}

//######################################################################
auto_ptr<AppLOS> app;
void AutoGL_SetUp(int argc, char* argv[]){
  app.reset(new AppLOS());
  app->init();
  app->run();
}
