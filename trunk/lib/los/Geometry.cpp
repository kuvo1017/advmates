#include "Geometry.h"
#include <cmath>
#include <iostream>

using namespace std;

//#define COLLISION_DEBUG

//======================================================================
double MyVector3D::length() const{
  return sqrt(_x*_x+_y*_y+_z*_z);
}

//======================================================================
void MyVector3D::normalize(){
  double l = length();
  _x /= l;
  _y /= l;
  _z /= l;
}

//======================================================================
double MyVector3D::dot(const MyVector3D vec) {
  return _x*vec.getX()+_y*vec.getY()+_z*vec.getZ();
}

//======================================================================
MyVector3D MyVector3D::cross(const MyVector3D vec) {
  return MyVector3D(_y*vec.getZ() - _z*vec.getY(),
		    _z*vec.getX() - _x*vec.getZ(),
		    _x*vec.getY() - _y*vec.getX());
}

//======================================================================
MyVector3D& MyVector3D::operator=(const MyVector3D& rhs) {
  _x = rhs._x;
  _y = rhs._y;
  _z = rhs._z;
  return *this;
}
MyVector3D& MyVector3D::operator+=(const MyVector3D& rhs) {
  _x += rhs._x;
  _y += rhs._y;
  _z += rhs._z;
  return *this;
}
MyVector3D& MyVector3D::operator-=(const MyVector3D& rhs) {
  _x -= rhs._x;
  _y -= rhs._y;
  _z -= rhs._z;
  return *this;
}
MyVector3D& MyVector3D::operator*=(double k) {
  _x *= k;
  _y *= k;
  _z *= k;
  return *this;
}

//======================================================================
MyVector3D operator+(const MyVector3D& lhs, const MyVector3D& rhs) {
  return (MyVector3D(lhs)+=rhs);
}
MyVector3D operator-(const MyVector3D& lhs, const MyVector3D& rhs) {
  return (MyVector3D(lhs)-=rhs);
}
MyVector3D operator*(double k, const MyVector3D& rhs){
  return (MyVector3D(rhs)*=k);
}
MyVector3D operator*(const MyVector3D& lhs, double k){
  return (MyVector3D(lhs)*=k);
}

//######################################################################
MyPolygon::MyPolygon(){
}

//======================================================================
MyPolygon::~MyPolygon(){
}

//======================================================================
MyVector3D MyPolygon::getVector(int i) const{
  return _vec[i];
}

//======================================================================
void MyPolygon::setVector(int i,
				double x, double y, double z){
  _vec[i].set(x,y,z); 
}

//======================================================================
void MyPolygon::setTriangle(double x0, double y0, double z0,
			    double x1, double y1, double z1,
			    double x2, double y2, double z2){
  _vec[0].set(x0,y0,z0);
  _vec[1].set(x1,y1,z1);
  _vec[2].set(x2,y2,z2);
}

//======================================================================
void MyPolygon::setTriangle(MyVector3D v0,
			    MyVector3D v1,
			    MyVector3D v2){
  _vec[0] = v0;
  _vec[1] = v1;
  _vec[2] = v2;
}

//======================================================================
double MyPolygon::judgeCollision(MyVector3D t, MyVector3D r){
  // 直線の方向ベクトル
  MyVector3D dv = r-t;
  // dv.normalize();

  // 平面の法線ベクトル
  MyVector3D nv = (_vec[1]-_vec[0]).cross(_vec[2]-_vec[0]);
  nv.normalize();

  double a = (t-_vec[0]).dot(nv);
  double b = dv.dot(nv);
  if (b==0.0) {
    // 法線ベクトルと方向ベクトルの内積が0であれば交点はない
    return -1.0;
  }

  // 媒介変数k
  double k = -a/b;
  if (k<=0 || k>=1) {
    // 線分と面は交点を持たない
    return -1.0;
  }

  // 平面上の点
  MyVector3D pv = t+k*dv;

  // これが三角形の中に入っていればよい
  for (int i=0; i<3; i++) {
    /*
    if ((_vec[i]-pv).cross(_vec[(i+1)%3]-pv).length()!=0) {
      cout << "here!!" << endl;
    }
    */
    double c = (_vec[i]-pv).cross(_vec[(i+1)%3]-pv).dot(nv);
    if (c<0)
      return -1.0;
  }

#ifdef COLLISION_DEBUG
  cout << " t:("   << t.getX()
       << ", "  << t.getY()
       << ", "  << t.getZ()
       << ")" << endl;
  cout << " r:(" << r.getX()
       << ", "  << r.getY()
       << ", "  << r.getZ() 
       << ")"   << endl; 
  cout << " d:(" << dv.getX()
       << ", "  << dv.getY()
       << ", "  << dv.getZ() 
       << ")"   << endl; 
  cout << " p:(" << pv.getX()
       << ", "  << pv.getY()
       << ", "  << pv.getZ() 
       << ")"   << endl; 
   for (int i=0; i<3; i++) {
    cout << " ("  << _vec[i].getX()
	 << ", " << _vec[i].getY()
	 << ", " << _vec[i].getZ()
	 << ")" << endl;
  }
  cout << " k=" << k << " " << endl;
#endif //COLLISION_DEBUG

  return k;
}
