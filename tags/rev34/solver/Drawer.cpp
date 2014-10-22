#include "Drawer.h"
#include "ObjManager.h"
#include "RoadMap.h"
#include "LaneBundle.h"
#include "Intersection.h"
#include "Section.h"
#include "Lane.h"
#include "Signal.h"
#include "SignalColor.h"
#include "RoadEntity.h"
#include "AmuLineSegment.h"
#include "Connector.h"
#include "AmuPoint.h"
#include "AmuVector.h"
#include "AmuConverter.h"
#include "DetectorUnit.h"
#include "Detector.h"
#include "Vehicle.h"
#include "VehicleFamily.h"
#include "Blinker.h"
#include "TimeManager.h"
#include "GLColor.h"
#include "Conf.h"
#include "GVManager.h"
#include "autogl_mates.h"
#include <math.h>
#include <autogl.h>
#include <iostream>
#include <string>
#include <sstream>
#include <map>
#include <vector>
#include <cassert>
#include <cstdlib>

//#define DIRECTION_DEBUG

#define ATTRIBUTE    0
#define ACCESS_RIGHT 1

#define CONNECTOR_ID_DISABLE 0
#define CONNECTOR_ID_GLOBAL  1
#define CONNECTOR_ID_LOCAL   2

using namespace std;

//######################################################################
RoadEntityDrawer& RoadEntityDrawer::instance()
{
  static RoadEntityDrawer instance;
  return instance;
}
//----------------------------------------------------------------------
void RoadEntityDrawer::draw(const RoadEntity& entity,
			    bool isSubsectionShape,
			    bool isSubnetwork,
			    bool isSubsectionId,
			    int surfaceMode,
			    bool isSignals) const
{
  if (isSubsectionShape)
  {
    drawShape(entity, surfaceMode);
  }
  if (isSubnetwork)
  {
    drawSubnetwork(entity, 0.2, 0.15);
  }
  if (entity.type()==SUBROAD_TYPE_CROSSWALK && isSignals)
  {
    drawSignals(entity);
  }
  if (isSubsectionId)
  {
    drawId(entity);
  }
}
//----------------------------------------------------------------------
void RoadEntityDrawer::drawShape(const RoadEntity& entity,
				 int surfaceMode) const
{
  SubroadFactory type = entity.type();

  if (surfaceMode == ATTRIBUTE)
  {
    // 描画属性で色分けする
    if (type==SUBROAD_TYPE_STREET)
    {
      GLColor::setSubsection();
    }
    else if (type==SUBROAD_TYPE_SIDEWALK)
    {
      GLColor::setSideWalk();
    }
    else if (type==SUBROAD_TYPE_CROSSWALK)
    {
      GLColor::setCrossWalk();
    }
    else
    {
      GLColor::setSubsection();
    }
  }
  else
  {
    // 通行権で色分けする
    if (entity.mayPass(TRAFFIC_VEHICLE_ANY)
	&& entity.mayPass(TRAFFIC_WALKER))
    {
      GLColor::setAnyTrafficAccessible();
    }
    else if (entity.mayPass(TRAFFIC_VEHICLE_ANY))
    {
      GLColor::setVehicleAccessible();
    }
    else if (entity.mayPass(TRAFFIC_WALKER))
    {
      GLColor::setPedestrianAccessible();
    }
    else
    {
      GLColor::setAnyTrafficInaccessible();
    }
  }

  // 凹型でも描けるように中点を使う
  AmuPoint p0 = entity.center();
  AmuPoint p1, p2;
  p1 = entity.vertex(entity.numVertices() - 1);
  for (int i = 0; i < entity.numVertices(); i++)
  {
    p2 = entity.vertex(i);
    AutoGL_DrawTriangle(p0.x(), p0.y(), p0.z(),
			p1.x(), p1.y(), p1.z(),
			p2.x(), p2.y(), p2.z());
    p1 = p2;
  }
}
//----------------------------------------------------------------------
void RoadEntityDrawer::drawSignals(const RoadEntity& entity) const
{
  for (int i=0; i<entity.numVertices(); i++)
  {
    if (entity.adjEntity(i) != NULL &&  // 一周分の歩道があるとは限らない
        entity.adjEntity(i)->type() == SUBROAD_TYPE_SIDEWALK)
    {
      AmuPoint point = entity.edge(i).createInteriorPoint(1,1);

      Signal* signal = entity.signal();
      int dir = entity.signalDirection();

      if (signal!=NULL)
      {
	if(signal->walkerColor(dir) == SignalColor::blue())
        {
	  GLColor::setBlueSignal();
	}
        else if(signal->walkerColor(dir) == SignalColor::red())
        {
	  GLColor::setRedSignal();
	}
        else if(signal->walkerColor(dir) == SignalColor::yellow())
        {
	  GLColor::setYellowSignal();
	}
        else
        {
	  AutoGL_SetColor(0,0,0);
	}	  
	AutoGL_DrawQuadrangle(point.x()-0.4, point.y(),     point.z()+4.0,
			      point.x(),     point.y()-0.4, point.z()+4.0,
			      point.x()+0.4, point.y(),     point.z()+4.0,
			      point.x(),     point.y()+0.4, point.z()+4.0);
      }
    }
  }
}
//----------------------------------------------------------------------
void RoadEntityDrawer::drawSubnetwork(const RoadEntity& entity,
				      double height,
				      double width) const
{
  GLColor::setSubnetwork();
  // サブセクションの中心点
  AutoGL_DrawCircle3D(entity.center().x(),
		      entity.center().y(),
		      entity.center().z()+height,
		      0, 0, 1, width*4, 3);
  // 中心点と、隣接するサブセクションの中心点を結ぶ
  for (int i=0;
       i<static_cast<signed int>(entity.numVertices());
       i++)
  {
    if (entity.adjEntity(i)!=NULL)
    {
      GLColor::setSubnetwork();
      AutoGL_DrawBoldLine2D(entity.center().x(),
			    entity.center().y(),
			    entity.center().z()+height*2,
			    entity.adjEntity(i)->center().x(),
			    entity.adjEntity(i)->center().y(),
			    entity.adjEntity(i)->center().z()+height*2,
			    width);
    }
    else
    {
      GLColor::setSubsectionEdge();
      AutoGL_DrawBoldLine2D(entity.edge(i).pointBegin().x(),
			    entity.edge(i).pointBegin().y(),
			    entity.edge(i).pointBegin().z()+height,
			    entity.edge(i).pointEnd().x(),
			    entity.edge(i).pointEnd().y(),
			    entity.edge(i).pointEnd().z()+height,
			    width);
    }
  }
}
//----------------------------------------------------------------------
void RoadEntityDrawer::drawId(const RoadEntity& entity) const
{
  GLColor::setSubsectionId();
  const AmuPoint center = entity.center();
  AutoGL_DrawString(center.x(), center.y(), 25,
		    entity.id().c_str());
}

//######################################################################
LaneDrawer& LaneDrawer::instance()
{
  static LaneDrawer instance;
  return instance;
}
//----------------------------------------------------------------------
void LaneDrawer::draw(const Lane& lane, double height, double width) const
{
  assert(lane.beginConnector());
  assert(lane.endConnector());

  AutoGL_DrawBoldArrow2D(width,
			 lane.beginConnector()->x(),
			 lane.beginConnector()->y(),
			 lane.beginConnector()->z()+height,
			 lane.endConnector()->x(),
			 lane.endConnector()->y(),
			 lane.endConnector()->z()+height);
}
//----------------------------------------------------------------------
void LaneDrawer::drawId(const Lane& lane) const
{
  GLColor::setLaneId();
  double x = (lane.beginConnector()->x()
	      + lane.endConnector()->x())*0.5;
  double y = (lane.beginConnector()->y()
	      + lane.endConnector()->y())*0.5;
  AutoGL_DrawString(x, y, 30,
		    lane.id().c_str());
}

//######################################################################
SignalDrawer& SignalDrawer::instance()
{
  static SignalDrawer instance;
  return instance;
}
//----------------------------------------------------------------------
void SignalDrawer::draw(const Intersection& inter) const
{
  const Signal* signal = inter.signal();
  if (signal!=NULL)
  {
    for(int i = 0;i < inter.numNext();i++)
    {
      if (inter.border(i))
      {
        for(int j = 0;j < inter.border(i)->numIn();j++)
        {
	  AmuPoint point = inter.border(i)->connector(j)->point();

          //上三角形の描画
          if(signal->mainColor(i) == SignalColor::blue())
          {
            GLColor::setBlueSignal();
          }
          else if(signal->mainColor(i) == SignalColor::red())
          {
            GLColor::setRedSignal();
          }
          else if(signal->mainColor(i) == SignalColor::yellow())
          {
            GLColor::setYellowSignal();
          }
          else if(signal->mainColor(i) == SignalColor::redblink())
          {
            if(TimeManager::time()%2000 < 1200) GLColor::setRedSignal();
            else AutoGL_SetColor(1,1,1);
          }
          else if(signal->mainColor(i) == SignalColor::yellowblink())
          {
            if(TimeManager::time()%2000 < 1200)
	    {
	      GLColor::setYellowSignal();
            }
	    else
	    {
	      AutoGL_SetColor(1,1,1);
	    }
	  }
          else
          {
            AutoGL_SetColor(0,0,0);
          }	  
	  AutoGL_DrawTriangle(point.x()+0.6, point.y()+0.6, point.z()+4.0,
			      point.x()+0.6, point.y()-0.6, point.z()+4.0,
			      point.x()-0.6, point.y()+0.6, point.z()+4.0);

          //下三角形の描画
          if(signal->subColor(i) == SignalColor::none())
          {
            GLColor::setNoneSignal();
          }
          else if(signal->subColor(i) == SignalColor::all())
          {
            GLColor::setAllSignal();
          }
          else if(signal->subColor(i) == SignalColor::straight())
          {
            GLColor::setStraightSignal();
          }
          else if(signal->subColor(i) == SignalColor::left())
          {
            GLColor::setLeftSignal();
          }
          else if(signal->subColor(i) == SignalColor::right())
          {
            GLColor::setRightSignal();
          }
          else if(signal->subColor(i) == SignalColor::straightLeft())
          {
            GLColor::setStraightLeftSignal();
          }
          else if(signal->subColor(i) == SignalColor::straightRight())
          {
            GLColor::setStraightRightSignal();
          }
          else if(signal->subColor(i) == SignalColor::leftRight())
          {
            GLColor::setLeftRightSignal();
          }
          else
          {
            AutoGL_SetColor(0,0,0);
          }
	  AutoGL_DrawTriangle(point.x()+0.6, point.y()-0.6, point.z()+4.0,
			      point.x()-0.6, point.y()-0.6, point.z()+4.0,
			      point.x()-0.6, point.y()+0.6, point.z()+4.0);
        }
      }
    }
  }
}

//######################################################################
IntersectionDrawer& IntersectionDrawer::instance()
{
  static IntersectionDrawer instance;
  return instance;
}
//----------------------------------------------------------------------
void IntersectionDrawer::draw(const Intersection& inter,
			      bool isSectionId,
			      bool isLanes,
			      bool isLaneId,
			      bool isSubsectionShape,
			      bool isSubnetwork,
			      bool isSubsectionId,
			      int surfaceMode,
			      bool isSignals,
			      int connectorIdMode) const
{
  drawSubsections(inter, 
		  isSubsectionShape, 
		  isSubnetwork,
		  isSubsectionId,
		  surfaceMode,
		  isSignals);
  if (isLanes)
  {
    drawLanes(inter, isLaneId);
  }
  if (connectorIdMode!=CONNECTOR_ID_DISABLE)
  {
    drawConnectors(inter, connectorIdMode);
  }
  if (isSectionId)
  {
    drawId(inter);
  }
  if (isSignals)
  {
    drawSignals(inter);
  }
}
//----------------------------------------------------------------------
void IntersectionDrawer::drawSimple(const Intersection& inter,
				    double radius,
				    bool isSectionId)  const
{
  GLColor::setSimpleNetwork();
  AutoGL_DrawCircle3D(inter.center().x(),
		      inter.center().y(),
		      inter.center().z(),
		      0,0,1,radius,3);

  if (isSectionId)
  {
    drawId(inter);
  }
}
//----------------------------------------------------------------------
void IntersectionDrawer::drawSubsections(const Intersection& inter,
					 bool isSubsectionShape,
					 bool isSubnetwork,
					 bool isSubsectionId,
					 int surfaceMode,
					 bool isSignals) const
{
  const map<string, RoadEntity*, less<string> >* entities
    = inter.entities();
  map<string, RoadEntity*, less<string> >::const_iterator cit;

  RoadEntityDrawer& roadEntityDrawer = RoadEntityDrawer::instance();
  for (cit=entities->begin(); cit!=entities->end(); cit++)
  {
    roadEntityDrawer.draw(*((*cit).second),
			  isSubsectionShape,
			  isSubnetwork,
			  isSubsectionId,
			  surfaceMode,
			  isSignals);
  }
}
//----------------------------------------------------------------------
void IntersectionDrawer::drawLanes(const Intersection& inter,
				   bool isLaneId) const
{
  const map<string, Lane*, less<string> >* lanes = inter.lanes();
  map<string, Lane*, less<string> >::const_iterator cit;

  LaneDrawer& laneDrawer = LaneDrawer::instance();
  for (cit=lanes->begin(); cit!=lanes->end(); cit++)
  {
#ifdef DIRECTION_DEBUG
    if (inter.isRight((*cit).second))
      GLColor::setRightLane();
    else if (inter.isLeft((*cit).second))
      GLColor::setLeftLane();
    else if (inter.isStraight((*cit).second))
      GLColor::setStraightLane();
    else
      GLColor::setLane();
#else
    GLColor::setLane();
#endif //DIREC_DEBUG
    laneDrawer.draw(*((*cit).second), 0.1, 0.15);
    if (isLaneId)
    {
      laneDrawer.drawId(*((*cit).second));
    }
  }
}
//----------------------------------------------------------------------
void IntersectionDrawer::drawConnectors(const Intersection& inter,
					int connectorIdMode) const
{
  const map<string, Connector*, less<string> >* connectors
    = inter.innerConnectors();
  map<string, Connector*, less<string> >::const_iterator cit;
  vector<Border*> borders = inter.borders();

  GLColor::setInterId();
  if (connectorIdMode==CONNECTOR_ID_GLOBAL)
  {
    // 内部コネクタ
    for (cit=connectors->begin(); cit!=connectors->end(); cit++)
    {
      AutoGL_DrawString
	((*cit).second->x(), (*cit).second->y(), 5,
	 (AmuConverter::itos((*cit).second->idGlobal(),
			     NUM_FIGURE_FOR_CONNECTOR_GLOBAL).c_str()));
    }
    // 境界コネクタのサブコンテナはBorderに含まれるが，
    // これはIntersectionの内部コネクタではない
    for (int i=0; i<static_cast<signed int>(borders.size()); i++)
    {
      for (int j=0; j<borders[i]->numIn()+borders[i]->numOut(); j++)
      {
	const Connector* bconnector = borders[i]->connector(j);
	AutoGL_DrawString
	  (bconnector->x(), bconnector->y(), 5,
	   (AmuConverter::itos(bconnector->idGlobal(),
			       NUM_FIGURE_FOR_CONNECTOR_GLOBAL).c_str()));
      }
    }
  }
  else if(connectorIdMode==CONNECTOR_ID_LOCAL)
  {
    // 内部コネクタ
    for (cit=connectors->begin(); cit!=connectors->end(); cit++)
    {
      AutoGL_DrawString((*cit).second->x(), (*cit).second->y(), 5,
			((*cit).first).c_str());
    }
    // 境界コネクタのサブコンテナはBorderに含まれるが，
    // これはIntersectionの内部コネクタではない
    for (int i=0; i<static_cast<signed int>(borders.size()); i++)
    {
      for (int j=0; j<borders[i]->numIn()+borders[i]->numOut(); j++)
      {
	const Connector* bconnector = borders[i]->connector(j);
	string localId
	  = "**"+AmuConverter::itos(j, NUM_FIGURE_FOR_CONNECTOR_LOCAL-2);
	AutoGL_DrawString
	  (bconnector->x(), bconnector->y(), 5, localId.c_str());
      }
    }
  }
}
//----------------------------------------------------------------------
void IntersectionDrawer::drawSignals(const Intersection& inter) const
{
  SignalDrawer& signalDrawer = SignalDrawer::instance();
  signalDrawer.draw(inter);
}
//----------------------------------------------------------------------
void IntersectionDrawer::drawId(const Intersection& inter) const
{
  GLColor::setInterId();
  const AmuPoint center = inter.center();
  AutoGL_DrawString(center.x(), center.y(), 30,
		    inter.id().c_str());
}

//######################################################################
SectionDrawer& SectionDrawer::instance()
{
  static SectionDrawer instance;
  return instance;
}
//----------------------------------------------------------------------
void SectionDrawer::draw(const Section& section,
			 bool isSectionId,
			 bool isLanes,
			 bool isLaneId,
			 bool isSubsectionShape,
			 bool isSubnetwork,
			 bool isSubsectionId,
			 int surfaceMode,
			 bool isSignals,
			 int connectorIdMode) const
{
  drawSubsections(section, 
		  isSubsectionShape,
		  isSubnetwork, 
		  isSubsectionId,
		  surfaceMode,
		  isSignals);
  if (isLanes)
  {
    drawLanes(section, isLaneId);
  }
  if (connectorIdMode!=CONNECTOR_ID_DISABLE)
  {
    drawConnectors(section, connectorIdMode);
  }
}
//----------------------------------------------------------------------
void SectionDrawer::drawSimple(const Section& section,
			       double width) const
{
  GLColor::setSimpleNetwork();
  AutoGL_DrawBoldLine2D(section.intersection(true)->center().x(),
			section.intersection(true)->center().y(),
			section.intersection(true)->center().z(),
			section.intersection(false)->center().x(),
			section.intersection(false)->center().y(),
			section.intersection(false)->center().z(),
			width);
}
//----------------------------------------------------------------------
void SectionDrawer::drawSubsections(const Section& section,
				    bool isSubsectionShape,
				    bool isSubnetwork,
				    bool isSubsectionId,
				    int surfaceMode,
				    bool isSignals) const
{
  const map<string, RoadEntity*, less<string> >* entities
    = section.entities();
  map<string, RoadEntity*, less<string> >::const_iterator cit;
  RoadEntityDrawer& roadEntityDrawer = RoadEntityDrawer::instance();

  for (cit=entities->begin(); cit!=entities->end(); cit++)
  {
    roadEntityDrawer.draw(*((*cit).second),
			  isSubsectionShape,
			  isSubnetwork,
			  isSubsectionId,
			  surfaceMode,
			  isSignals);
  }
}
//----------------------------------------------------------------------
void SectionDrawer::drawLanes(const Section& section,
			      bool isLaneId) const
{
  const map<string, Lane*, less<string> >* lanes = section.lanes();
  map<string, Lane*, less<string> >::const_iterator cit;

  LaneDrawer& laneDrawer = LaneDrawer::instance();
  for (cit=lanes->begin(); cit!=lanes->end(); cit++)
  {
#ifdef DIRECTION_DEBUG
    if (section.isUp((*cit).second))
      GLColor::setUpLane();
    else
      GLColor::setDownLane();
#else
    GLColor::setLane();
#endif //DIREC_DEBUG
    laneDrawer.draw(*((*cit).second), 0.1, 0.15);
    if (isLaneId)
    {
      laneDrawer.drawId(*((*cit).second));
    }
  }
}
//----------------------------------------------------------------------
void SectionDrawer::drawConnectors(const Section& section,
				   int connectorIdMode) const
{
  const map<string, Connector*, less<string> >* connectors
    = section.innerConnectors();
  map<string, Connector*, less<string> >::const_iterator cit;

  GLColor::setInterId();
  if (connectorIdMode==CONNECTOR_ID_GLOBAL)
  {
    // 内部コネクタ
    for (cit=connectors->begin(); cit!=connectors->end(); cit++)
    {
      AutoGL_DrawString
	((*cit).second->x(), (*cit).second->y(), 5,
	 (AmuConverter::itos((*cit).second->idGlobal(),
			     NUM_FIGURE_FOR_CONNECTOR_GLOBAL).c_str()));
    }
  }
  else if(connectorIdMode==CONNECTOR_ID_LOCAL)
  {
    // 内部コネクタ
    for (cit=connectors->begin(); cit!=connectors->end(); cit++)
    {
      AutoGL_DrawString((*cit).second->x(), (*cit).second->y(), 5,
			((*cit).first).c_str());
    }
  }
}

//######################################################################
RoadMapDrawer& RoadMapDrawer::instance()
{
  static RoadMapDrawer instance;
  return instance;
}
//----------------------------------------------------------------------
RoadMapDrawer::RoadMapDrawer()
{
  _intersectionDrawer = &IntersectionDrawer::instance();
  _sectionDrawer = &SectionDrawer::instance();
}
//----------------------------------------------------------------------
void RoadMapDrawer::draw(const RoadMap& roadMap,
			 bool isSimpleMap,
			 bool isSectionId,
			 bool isLanesInter,
			 bool isLanesSection,
			 bool isLaneId,
			 bool isSubsectionShape,
			 bool isSubnetwork,
			 bool isSubsectionId,
			 int surfaceMode,
			 bool isSignals,
			 int connectorIdMode) const
{
  // 通常の地図描画
  if (!isSimpleMap)
  {
    const RMAPI* intersections = roadMap.intersections();
    CITRMAPI iti = (*intersections).begin();
    while (iti!=(*intersections).end()) {
      _intersectionDrawer->draw(*((*iti).second), 
				isSectionId,
				isLanesInter,
				isLaneId,
				isSubsectionShape,
				isSubnetwork,
				isSubsectionId,
				surfaceMode,
				isSignals,
				connectorIdMode);
      iti++;
    }
  
    const RMAPS* sections = roadMap.sections();
    CITRMAPS its = (*sections).begin();
    while (its!=(*sections).end()) {
      _sectionDrawer->draw(*((*its).second),
			   isSectionId,
			   isLanesSection,
			   isLaneId,
			   isSubsectionShape,
			   isSubnetwork,
			   isSubsectionId,
			   surfaceMode,
			   isSignals,
			   connectorIdMode);
      its++;
    }
  }

  // シンプルな地図描画
  else
  {
    // リンク幅を決定する(viewsizeの0.5%)
    double linkWidth = 0.005 * AutoGL_GetViewSize();

    // 交差点を描画
    const RMAPI* intersections = roadMap.intersections();
    CITRMAPI iti = (*intersections).begin();
    while (iti!=(*intersections).end())
    {
      _intersectionDrawer->drawSimple(*((*iti).second),
				      linkWidth,
				      isSectionId);
      iti++;
    }

    // 道路を描画(両端を結ぶ幅linkWidthの線分)
    const RMAPS* sections = roadMap.sections();
    CITRMAPS its = (*sections).begin();
    while (its!=(*sections).end())
    {
      _sectionDrawer->drawSimple(*((*its).second),
				 linkWidth);
      its++;
    }
  }
}

//######################################################################
DetectorDrawer& DetectorDrawer::instance()
{
  static DetectorDrawer instance;
  return instance;
}
//----------------------------------------------------------------------
void DetectorDrawer::draw(const DetectorUnit& unit) const
{
  const vector<Detector*>* detectors = unit.detectors();
  for (int i=0; i<static_cast<signed int>(detectors->size()); i++)
  {
    double x = (*detectors)[i]->x();
    double y = (*detectors)[i]->y();
    double z = (*detectors)[i]->z()+0.2;
    double size = 1.5;
    AmuVector vec0 = (*detectors)[i]->lane()->directionVector();
    vec0.normalize();
    AmuVector vec1 = vec0;
    vec1.revoltXY(M_PI_2);

    // 左三角の描画
    GLColor::setDetector();
    AutoGL_DrawTriangle(x,y,z,
			x+vec0.x()*size*0.5+vec1.x()*size,
			y+vec0.y()*size*0.5+vec1.y()*size, z,
			x-vec0.x()*size*0.5+vec1.x()*size,
			y-vec0.y()*size*0.5+vec1.y()*size, z);
    // IDの描画
    if (i==0)
    {
      GLColor::setRoadsideUnitId();
      AutoGL_DrawString(x, y, 5, unit.id().c_str());
    }
  }
}

//######################################################################
VehicleDrawer& VehicleDrawer::instance()
{
  static VehicleDrawer instance;
  return instance;
}
//----------------------------------------------------------------------
void VehicleDrawer::draw(const Vehicle& vehicle, 
			 bool isVehicleId) const
{
  double x = vehicle.x();
  double y = vehicle.y();
  double z = vehicle.z();
  double bodyLength = vehicle.bodyLength();
  double bodyWidth = vehicle.bodyWidth();
  double bodyHeight = vehicle.bodyHeight();
  AmuVector v0 = vehicle.directionVector();
  v0.normalize();
  AmuVector v1 = v0;
  v1.revoltXY(M_PI_2);

  double x0,x1,x2,x3,y0,y1,y2,y3;
  double z0 = z+bodyHeight;

  x0 = x + bodyLength / 2 * v0.x() + bodyWidth / 2 * v1.x();
  y0 = y + bodyLength / 2 * v0.y() + bodyWidth / 2 * v1.y();

  x1 = x + bodyLength / 2 * v0.x() - bodyWidth / 2 * v1.x();
  y1 = y + bodyLength / 2 * v0.y() - bodyWidth / 2 * v1.y();

  x2 = x - bodyLength / 2 * v0.x() - bodyWidth / 2 * v1.x();
  y2 = y - bodyLength / 2 * v0.y() - bodyWidth / 2 * v1.y();

  x3 = x - bodyLength / 2 * v0.x() + bodyWidth / 2 * v1.x();
  y3 = y - bodyLength / 2 * v0.y() + bodyWidth / 2 * v1.y();

  double r, g, b;
  vehicle.getBodyColor(&r, &g, &b);
  AutoGL_SetColor(r, g, b);
  AutoGL_DrawQuadrangle(x0, y0, z0,
			x1, y1, z0,
			x2, y2, z0,
			x3, y3, z0);

  if (TimeManager::time()%2000<1000)
  {
    double zBlink = 0.2;
    AutoGL_SetColor(1,1,0);
    if (vehicle.blinker().isRight())
    {
      AutoGL_DrawCircle3D(x1, y1, z0+zBlink, 0, 0, 1, 1, 3);
    }
    else if (vehicle.blinker().isLeft())
    {
      AutoGL_DrawCircle3D(x0, y0, z0+zBlink, 0, 0, 1, 1, 3);
    }
  }

  string id;
  GLColor::setVehicleId();
  if (isVehicleId)
  {
    id = vehicle.id();
  }
  if (id != "")
  {
    AutoGL_DrawString(x, y, z+5, id.c_str());
  }
}
//----------------------------------------------------------------------
void VehicleDrawer::drawSimple(const Vehicle& vehicle, 
			       bool isVehicleId,
			       double size) const
{
  double x = vehicle.x();
  double y = vehicle.y();
  double z = vehicle.z();
  AmuVector v0 = vehicle.directionVector();
  v0.normalize();
  AmuVector v1 = v0;
  v1.revoltXY(M_PI_2);

  double x0,x1,x2,y0,y1,y2;
  double z0 = z+2;

  x0 = x + size * v0.x();
  y0 = y + size * v0.y();

  x1 = x - size * v0.x() + size * v1.x();
  y1 = y - size * v0.y() + size * v1.y();

  x2 = x - size * v0.x() - size * v1.x();
  y2 = y - size * v0.y() - size * v1.y();

  GLColor::setVehicle();
  AutoGL_DrawTriangle(x0, y0, z0,
		      x1, y1, z0,
		      x2, y2, z0);
  if (isVehicleId)
  {
    GLColor::setVehicleId();
    AutoGL_DrawString(x, y, z+5, vehicle.id().c_str());
  }
}

