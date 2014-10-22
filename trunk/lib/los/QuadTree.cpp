#include "QuadTree.h"
#include "Geometry.h"
#include "GeoManager.h"
#include "LOSConf.h"
#include <iostream>
#include <iomanip>
#include <cstdlib>

//#define QT_DEBUG
//#define CN_DEBUG
//#define COLLISION_DEBUG

//======================================================================
QuadTree::QuadTree(){
  for (int i=0; i<4; i++) {
    _child[i] = NULL;
  }
}

//======================================================================
QuadTree::QuadTree(int depth, int id){
  for (int i=0; i<4; i++) {
    _child[i] = NULL;
  }
  _depth = depth;
  _id    = id;
}

//======================================================================
QuadTree::~QuadTree(){
}

//======================================================================
QuadTree* QuadTree::child(int i){
  return _child[i];
}

//======================================================================
void QuadTree::pushWall(MyPolygon* wall, int pid){
  _walls.push_back(wall);
  _pids.push_back(pid);
}

//======================================================================
void QuadTree::pushQuadTree(MyPolygon* wall, int pid, int depth,
			    vector<QuadTree*>* trees){
  if (depth < MAX_DEPTH) {
    int wx1, wx2, wx3, wy1, wy2, wy3;
    wx1 = xpos(wall->getVector(0));
    wx2 = xpos(wall->getVector(1));
    wx3 = xpos(wall->getVector(2));
    wy1 = ypos(wall->getVector(0));
    wy2 = ypos(wall->getVector(1));
    wy3 = ypos(wall->getVector(2));

#ifdef QT_DEBUG
    cout << "FIELD:" << endl
	 << "  " <<  _xmin
	 << ", " << (_xmin+_xmax)*0.5
	 << ", " <<  _xmax << endl
	 << "  " <<  _ymin
	 << ", " << (_ymin+_ymax)*0.5
	 << ", " <<  _ymax << endl;

    cout << "POLYGON:" << endl
	 << "  " << wall->getVector(0).getX()
	 << ", " << wall->getVector(0).getY()
	 << " in area " << wx1+wy1*2 << endl
	 << "  " << wall->getVector(1).getX()
	 << ", " << wall->getVector(1).getY()
	 << " in area " << wx2+wy2*2 << endl
	 << "  " << wall->getVector(2).getX()
	 << ", " << wall->getVector(2).getY()
	 << " in area " << wx3+wy3*2 << endl;
#endif

    if (wx1==wx2 && wx2==wx3 && wx1 <=1 &&
	wy1==wy2 && wy2==wy3 && wy1 <=1) {
      // 子ノードへの登録を試みる
      if (_child[0]==NULL) {
#ifdef QT_DEBUG
	cout << "make child." << endl;
#endif
	// 分割されていなければ分割
 	setChildTree(depth+1, _id, trees);
      }
      // 分割->登録は深さがMAX_DEPTHになるまで再帰的に行われる
      _child[wx1+wy1*2]->pushQuadTree(wall, pid, depth+1, trees);
    }
    else {
#ifdef QT_DEBUG
      cout << "Polygon :" << setw(5) << pid 
	   <<" - Push wall (1) to:" << setw(5) << _id
	   << "/" << setw(2) << _depth << endl;
#endif
      pushWall(wall, pid);
    }
  }
  else {
#ifdef QT_DEBUG
      cout << "Polygon :" << setw(5) << pid 
	   <<" - Push wall (2) to:" << setw(5) << _id
	 << "/" << setw(2) << _depth << endl;
#endif
    pushWall(wall, pid);
  }
}

//======================================================================
void QuadTree::setChildTree(int depth, int id, 
			    vector<QuadTree*>* trees){
  for (int i=0; i<4; i++) {
    QuadTree* cq = new QuadTree(depth, id*4+i);
    switch (i) {
    case 0:
      cq->set(_xmin, (_xmin+_xmax)*0.5, _ymin, (_ymin+_ymax)*0.5);
      break;
    case 1:
      cq->set((_xmin+_xmax)*0.5, _xmax, _ymin, (_ymin+_ymax)*0.5);
      break;
    case 2:
      cq->set(_xmin, (_xmin+_xmax)*0.5, (_ymin+_ymax)*0.5, _ymax);
      break;
    case 3:
      cq->set((_xmin+_xmax)*0.5, _xmax, (_ymin+_ymax)*0.5, _ymax);
      break;
    default:
      exit(EXIT_FAILURE);
    }
    _child[i] = cq;
    trees->push_back(cq);
#ifdef QT_DEBUG
    cout << "  depth:" << cq->depth() 
	 << " id:" << cq->id()
	 << " is child of " << id
	 << " (" << cq->xmin() << "," << cq->ymin() << ")"
	 << "-(" << cq->xmax() << "," << cq->ymax() << ")" << endl;
#endif
  }
}

//======================================================================
void QuadTree::checkNode(MyVector3D t, MyVector3D r, int depth, ChkTree* ct,
			 double a, double b){
  if (depth<MAX_DEPTH) {
    int tx,ty,rx,ry;
    tx = xpos(t);
    ty = ypos(t);
    rx = xpos(r);
    ry = ypos(r);
    
    if (tx==rx && ty==ry) {
      // tとrは同一の四分領域に含まれている
      if(_child[tx+ty*2]!=NULL) {
	ct->setChild();
	// 送信点と受信点は同じ子ノードに含まれる
	// 子ノードで再度チェックを行う
	_child[tx+ty*2]->checkNode(t, r, depth+1, ct->child(tx+ty*2), a, b);
      }
      else {
	// 最も小さなノード
	ct->setFlag(2);
      }
    }
    else if (tx==rx && ty!=ry) {
      // tとrは縦に並んだ領域に含まれている
      MyVector3D cp1, cp2, cp3, cp4;
      createControlPoint(&cp1, &cp2, &cp3, &cp4,
			 (_xmin+_xmax)*0.5, (_ymin+_ymax)*0.5, a, b);
#ifdef CN_DEBUG
      cout << "Point: "
	   << "(" << t.getX() << ", " << t.getY() << ")" << endl;
      cout << "Point: "
	   << "(" << r.getX() << ", " << r.getY() << ")" << endl;
      cout << "  tx=" << tx
	   << ", ty=" << ty
	   << ", rx=" << rx
	   << ", ry=" << ry << endl;
      cout << "  " << cp1.getX() << "," << cp1.getY() << endl
	   << "  " << cp2.getX() << "," << cp2.getY() << endl
	   << "  " << cp3.getX() << "," << cp3.getY() << endl
	   << "  " << cp4.getX() << "," << cp4.getY() << endl;
#endif
      // 下がtransmitになるよう入れ替え
      if (t.getY()>r.getY()) {
	MyVector3D swp = t;
	t = r;
	r = swp;
      }
      if (_child[tx]||_child[tx+2]) {
	ct->setChild();
	if (_child[tx]) 
	  _child[tx]->checkNode(t, cp4, depth+1, ct->child(tx), a, b);
	if (_child[tx+2])
	  _child[tx+2]->checkNode(cp2, r, depth+1, ct->child(tx+2), a, b);
      }
      else  {
	ct->setFlag(2);
      }
    }
    else if (tx!=rx && ty==ry) {
      // tとrは横に並んだ領域に含まれている
      MyVector3D cp1, cp2, cp3, cp4;
      createControlPoint(&cp1, &cp2, &cp3, &cp4,
			 (_xmin+_xmax)*0.5, (_ymin+_ymax)*0.5, a, b);
#ifdef CN_DEBUG
      cout << "Point: "
	   << "(" << t.getX() << ", " << t.getY() << ")" << endl;
      cout << "Point: "
	   << "(" << r.getX() << ", " << r.getY() << ")" << endl;
      cout << "  tx=" << tx
	   << ", ty=" << ty
	   << ", rx=" << rx
	   << ", ry=" << ry << endl;
      cout << "  " << cp1.getX() << "," << cp1.getY() << endl
	   << "  " << cp2.getX() << "," << cp2.getY() << endl
	   << "  " << cp3.getX() << "," << cp3.getY() << endl
	   << "  " << cp4.getX() << "," << cp4.getY() << endl;
#endif
      // 左がtransmitになるよう入れ替え
      if (t.getX()>r.getX()) {
	MyVector3D swp = t;
	t = r;
	r = swp;
      }
      if (_child[2*ty]||_child[2*ty+1]) {
	ct->setChild();
	if (_child[2*ty])
	  _child[2*ty]->checkNode(t, cp1, depth+1, ct->child(2*ty), a, b);
	if (_child[2*tx+1])
	  _child[2*tx+1]->checkNode(cp3, r, depth+1, ct->child(2*ty+1), a, b);
      }
      else {
	ct->setFlag(2);
      }
    }
    else {
      // tとrは斜めに並んだ領域に含まれている
      // (3領域を通過している)
      MyVector3D cp1, cp2, cp3, cp4;
      createControlPoint(&cp1, &cp2, &cp3, &cp4,
			 (_xmin+_xmax)*0.5, (_ymin+_ymax)*0.5, a, b);
#ifdef CN_DEBUG
      cout << "Point: "
	   << "(" << t.getX() << ", " << t.getY() << ")" << endl;
      cout << "Point: "
	   << "(" << r.getX() << ", " << r.getY() << ")" << endl;
      cout << "  tx=" << tx
	   << ", ty=" << ty
	   << ", rx=" << rx
	   << ", ry=" << ry << endl;
      cout << "  " << cp1.getX() << "," << cp1.getY() << endl
	   << "  " << cp2.getX() << "," << cp2.getY() << endl
	   << "  " << cp3.getX() << "," << cp3.getY() << endl
	   << "  " << cp4.getX() << "," << cp4.getY() << endl;
#endif
      // 左がtransmitになるよう入れ替え
      if (t.getX()>r.getX()) {
	MyVector3D swp = t;
	t = r;
	r = swp;
      }
      tx = xpos(t);
      ty = ypos(t);
      rx = xpos(cp1);
      ry = ypos(cp1);
      if (tx==rx && ty==ry) {
	if (ty==0) {
	  //t->cp1,cp3->cp4,cp2->r
	  if (_child[0]||_child[1]||_child[3]) {
	    ct->setChild();
	    if (_child[0])
	      _child[0]->checkNode(t, cp1, depth+1, ct->child(0), a, b);
	    if (_child[1])
	      _child[1]->checkNode(cp3, cp4, depth+1, ct->child(1), a, b);
	    if (_child[3])
	      _child[3]->checkNode(cp2, r, depth+1, ct->child(3), a, b);
	  }
	  else {
	    ct->setFlag(2);
	  }
	}
	else {
	  //t->cp1,cp3->cp2,cp4->r
	  if (_child[2]||_child[3]||_child[1]) {
	    ct->setChild();
	    if (_child[2])
	      _child[2]->checkNode(t, cp1, depth+1, ct->child(2), a, b);
	    if (_child[3])
	      _child[3]->checkNode(cp3, cp2, depth+1, ct->child(3), a, b);
	    if (_child[1])
	      _child[1]->checkNode(cp4, r, depth+1, ct->child(1), a, b);
	  }
	  else {
	    ct->setFlag(2);
	  }
	}
      }
      else {
	if (ty==0) {
	  //t->cp4,cp2->cp1,cp3->r
	  if (_child[0]||_child[2]||_child[3]) {
	    ct->setChild();
	    if (_child[0])
	      _child[0]->checkNode(t, cp4, depth+1, ct->child(0), a, b);
	    if (_child[2])
	      _child[2]->checkNode(cp2, cp1, depth+1, ct->child(2), a, b);
	    if (_child[3])
	      _child[3]->checkNode(cp3, r, depth+1, ct->child(3), a, b);
	  }
	  else {
	    ct->setFlag(2);
	  }
	}	
	else {
	  //t->cp2,cp4->cp1,cp3->r
	  if (_child[2]||_child[0]||_child[1]) {
	    ct->setChild();
	    if (_child[2])
	      _child[2]->checkNode(t, cp2, depth+1, ct->child(2), a, b);
	    if (_child[0])
	      _child[0]->checkNode(cp4, cp1, depth+1, ct->child(0), a, b);
	    if (_child[1])
	      _child[1]->checkNode(cp3, r, depth+1, ct->child(1), a, b);
	  }
	  else {
	    ct->setFlag(2);
	  }
	}
      }
    }
  }
}

//======================================================================
void QuadTree::countCollision(MyVector3D t, MyVector3D r, ChkTree* ct,
			      int* collisions, double* arrayT){
  for (unsigned int i=0; i<_walls.size(); i++) {
    double tc = _walls[i]->judgeCollision(t, r);
    if (tc>0.0) {
      // 交点あり
      *collisions = (*collisions)+1;
      arrayT[(*collisions)] = tc;
#ifdef COLLISION_DEBUG
      cout << "collision detected: " << _pids[i] 
	   << " in id:" << setw(5) << _id << endl;
#endif //COLLISION_DEBUG
    }
  }
  if (ct->flag()!=2) {
    // 子要素に続ける
    for (int j=0; j<4; j++) {
      if (_child[j]&&ct->child(j)) {
	_child[j]->countCollision(t, r, ct->child(j), collisions, arrayT);
      }
    }
  }
}

//======================================================================
void QuadTree::countCollision(LOSVehicle* tv, LOSVehicle* rv, ChkTree* ct,
			      int* collisions, double* arrayT){
  for (unsigned int i=0; i<_walls.size(); i++) {
    // 自分自身あるいは相手自身の車体ポリゴンは障害物とみなさない
    if (_pids[i]/10==tv->id() || _pids[i]/10==rv->id())
      continue;

    double tc = _walls[i]->judgeCollision(tv->vec(), rv->vec());
    if (tc>0.0) {
      // 交点あり
      *collisions = (*collisions)+1;
      arrayT[(*collisions)] = tc;
#ifdef COLLISION_DEBUG
      cout << "collision detected: " << _pids[i] 
	   << " in id:" << setw(5) << _id << endl;
#endif //COLLISION_DEBUG
    }
  }
  if (ct->flag()!=2) {
    // 子要素に続ける
    for (int j=0; j<4; j++) {
      if (_child[j]&&ct->child(j)) {
	_child[j]->countCollision(tv, rv, ct->child(j), collisions, arrayT);
      }
    }
  }
}

//======================================================================
int QuadTree::xpos(MyVector3D vec){
  if (vec.getX()<(_xmin+_xmax)*0.5)
    return 0;
  else 
    return 1;
}

//======================================================================
int QuadTree::ypos(MyVector3D vec){
  if (vec.getY()<(_ymin+_ymax)*0.5)
    return 0;
  else
    return 1;
}

//======================================================================
void QuadTree::createControlPoint(MyVector3D *cp1,
				  MyVector3D *cp2,
				  MyVector3D *cp3,
				  MyVector3D *cp4,
				  double x, double y, double a, double b){
  double vx = (y - b) / a;
  double vy = (a * x) + b;
  cp1->set(x-1, vy, 0);
  cp2->set(vx, y+1, 0);
  cp3->set(x+1, vy, 0);
  cp4->set(vx, y-1, 0);
}
