#ifndef __VISUALIZER_H__
#define __VISUALIZER_H__

class LOSSimulator;
class QuadTree;

class Visualizer{
 public:
  Visualizer(){};
  ~Visualizer();

  // LOSマネージャをセットする
  void setLosSimulator(LOSSimulator* los);

  // 可視化を開始する
  void visualize();

 private:
  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // コールバック関数

  // 描画する
  static void viewRedrawCallback();

  // Drawボタン
  static void drawButtonCallback();

  // 四分木領域の描画
  static void drawQuadTree(QuadTree* qt, double width, double height);

  // 幅のある線の描画
  static void AutoGL_DrawBoldLine2D(double x0, double y0, double z0,
				    double x1, double y1, double z1,
				    double width);

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

  // 描画対象のデータを持つLOSマネージャ
  static LOSSimulator* _los;

  // 領域
  static double _xmin, _xmax, _ymin, _ymax;

  //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  // 着色するポリゴンID
  static int _polygonId;

  // 可視化を制御する変数
  static int _showMesh;
  static int _showVehicleMesh;
  static int _showVehicleIds;

};
#endif //__VISUALIZER_H__
