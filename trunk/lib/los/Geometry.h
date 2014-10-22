#ifndef __GEOMETRY_H__
#define __GEOMETRY_H__

#include <cmath>

//######################################################################
// 3次元ベクトルを表すクラス
class MyVector3D{
 public:
  MyVector3D(){};
  MyVector3D(double x, double y, double z){_x=x;_y=y;_z=z;};
  ~MyVector3D(){};

  double getX() const{return _x;};
  double getY() const{return _y;};
  double getZ() const{return _z;};
  void get(double *x, double *y, double *z) const{*x=_x;*y=_y;*z=_z;};

  void setX(double x) {_x=x;};
  void setY(double y) {_y=y;};
  void setZ(double z) {_z=z;};
  void set(double x, double y, double z){_x=x;_y=y;_z=z;};

  double length() const;
  void normalize();

  double dot(const MyVector3D vec);
  MyVector3D cross(const MyVector3D vec);

  // 演算子
  MyVector3D& operator=(const MyVector3D& rhs);
  MyVector3D& operator+=(const MyVector3D& rhs);
  MyVector3D& operator-=(const MyVector3D& rhs);
  MyVector3D& operator*=(double k);

 private:
  double _x;
  double _y;
  double _z;
};

// 二項演算子
MyVector3D operator+(const MyVector3D& lhs, const MyVector3D& rhs);
MyVector3D operator-(const MyVector3D& lhs, const MyVector3D& rhs);
MyVector3D operator*(double k, const MyVector3D& rhs);
MyVector3D operator*(const MyVector3D& lhs, double k);

//######################################################################
// ポリゴン(三角形?)を表すクラス
class MyPolygon{
 public:
  MyPolygon();
  ~MyPolygon();

  MyVector3D getVector(int i) const;

  void setVector(int i, double x, double y, double z);
  void setTriangle(double x0, double y0, double z0,
		   double x1, double y1, double z1,
		   double x2, double y2, double z2);
  void setTriangle(MyVector3D v0, MyVector3D v1, MyVector3D v2);

  // 衝突判定
  double judgeCollision(MyVector3D t, MyVector3D r);

 private:
  // 位置ベクトル
  MyVector3D _vec[3];
};

//######################################################################
// 車両位置と車体サイズを格納するクラス
class LOSVehicle{
 public:
  LOSVehicle(){};
  ~LOSVehicle(){};

  int id()   const {return _id;};
  double x() const {return _x;};
  double y() const {return _y;};
  double z() const {return _z;};
  double length() const {return _length;};
  double width()  const {return _width;};
  double height() const {return _height;};

  void setAttribute(int id, double x, double y, double z,
		    double length, double width, double height){
    _id = id;
    _x = x;
    _y = y;
    _z = z;
    _length = length;
    _width  = width;
    _height = height;
  };
  MyVector3D vec() const{
    return MyVector3D(_x, _y, _z);
  };
  
 private:
  int _id;
  double _x,_y,_z;
  double _length,_width,_height;
};

//######################################################################
// 車体面を格納するクラス
class VehicleWall{
 public:
  VehicleWall(){};
  ~VehicleWall(){};

  MyPolygon wall(int i) const{return _wall[i];};
  void setPolygon(int i, MyVector3D v0, MyVector3D v1, MyVector3D v2) {
    _wall[i].setTriangle(v0,v1,v2);
  };
  void setPolygon(int i,
		  double x0, double y0, double z0,
		  double x1, double y1, double z1,
		  double x2, double y2, double z2) {
    _wall[i].setTriangle(x0, y0, z0,
			 x1, y1, z1,
			 x2, y2, z2);
  };

 private:
  MyPolygon _wall[10];
};

#endif //__GEOMETRY_H__
