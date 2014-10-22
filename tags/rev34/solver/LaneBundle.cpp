#include "LaneBundle.h"
#include "RoadEntity.h"
#include "Lane.h"
#include "Connector.h"
#include "AmuPoint.h"
#include "AmuLineSegment.h"
#include "RoadOccupant.h"
#include "Vehicle.h"
#include <algorithm>
#include <cassert>
#include <functional>
#include <string>

using namespace std;

//======================================================================
LaneBundle::LaneBundle(const string& id)
{
    _id = id;
}

//======================================================================
LaneBundle::~LaneBundle(){}

//======================================================================
int LaneBundle::numVertices() const
{
    return static_cast<signed int>(_vertices.size());
}

//======================================================================
const AmuPoint LaneBundle::vertex(int i) const
{
    assert(0<=i && i<static_cast<signed int>(_vertices.size()));
    return _vertices[i];
}

//======================================================================
void LaneBundle::addVertex(AmuPoint vertex)
{
    _vertices.push_back(vertex);
}

//======================================================================
const AmuLineSegment LaneBundle::edge(int i) const
{
    return AmuLineSegment(vertex(i), vertex((i+1)%numVertices()));
}

//======================================================================
const string& LaneBundle::id() const
{
    return _id;
}

//======================================================================
double LaneBundle::lengthToNext(const Lane* lane, double length) const
{
    assert(isMyLane(lane));

    double result = 0;
    const Lane* target = lane;
    result += target->length()-length;
    while (isNextLaneMine(target))
    {
        target = mostStraightNextLane(target);
        result += target->length();
    }
    return result;
}

//======================================================================
double LaneBundle::lengthFromPrevious(const Lane* lane, double length) const
{
    assert(isMyLane(lane));

    double result = 0;
    const Lane* target = lane;
    result += length;
    while (isPreviousLaneMine(target))
    {
        target = mostStraightPreviousLane(target);
        result += target->length();
    }
    return result;
}

//======================================================================
const map<string, Connector*, less<string> >* LaneBundle::innerConnectors() const
{
    return &_innerConnectors;
}

//======================================================================
const Connector* LaneBundle::connector(const string& id) const
{
    map<string, Connector*, less<string> >::const_iterator itc
        = _innerConnectors.find(id);
    if (itc!=_innerConnectors.end())
    {
        return (*itc).second;
    }
    else
    {
        cerr << "no lane bundle connector error " << _id << " " << id << endl;
        return 0;
    }
}

//======================================================================
const map<string, RoadEntity*, less<string> >* LaneBundle::entities() const
{
    return &_entities;
}

//======================================================================
bool LaneBundle::isMyRoadEntity(const RoadEntity* entity) const
{
    bool flag = false;
    map<string, RoadEntity*, less<string> >::const_iterator ite;
    for (ite=_entities.begin(); ite!=_entities.end(); ite++)
    {
        if ((*ite).second==entity)
        {
            flag = true;
            break;
        }
    }
    return flag;
}


//======================================================================
RoadEntity* LaneBundle::streetEntity()
{
    return _entities["00"];
}

//======================================================================
const map<string, Lane*, less<string> >* LaneBundle::lanes() const
{
    return &_lanes;
}

//======================================================================
bool LaneBundle::isMyLane(const Lane* lane) const
{
    if (!lane)
    {
        cout << __FUNCTION__ << ": lane is null." << _id << endl;
    }

    bool flag = false;
    map<string, Lane*, less<string> >::const_iterator itl
        = _lanes.find(lane->id());
    if (itl!=_lanes.end())
    {
        if (lane==(*itl).second)
        {
            flag = true;
        }
    }
    return flag;
}
//======================================================================
bool LaneBundle::isNextLaneMine(const Lane* lane) const
{
    bool flag = false;
    if (isMyLane(lane))
    {
        if (isMyLane(nextLane(lane,0)))
        {
            flag = true;
        }
    }
    return flag;
}

//======================================================================
bool LaneBundle::isPreviousLaneMine(const Lane* lane) const
{
    bool flag = false;
    if (isMyLane(lane))
    {
        if (isMyLane(previousLane(lane,0)))
        {
            flag = true;
        }
    }
    return flag;
}

//======================================================================
int LaneBundle::numNextLane(const Lane* lane) const
{
    return static_cast<signed int>(nextLanes(lane).size());
}

//======================================================================
Lane* LaneBundle::nextLane(const Lane* lane, int num) const
{
    assert(0<=num && num<numNextLane(lane));
    return (nextLanes(lane))[num];
}

//======================================================================
Lane* LaneBundle::shortestNextLane(const Lane* lane) const
{
    if (numNextLane(lane)==0)
        return NULL;
    vector<Lane*> lanes = nextLanes(lane);
    Lane* result = lanes[0];
    double minLength = lanes[0]->length();
    for (int i=1; i<static_cast<signed int>(lanes.size()); i++)
    {
        if (lanes[i]->length()<minLength)
        {
            result = lanes[i];
            minLength = lanes[i]->length();
        }
    }
    return result;
}

//======================================================================
Lane* LaneBundle::mostStraightNextLane(const Lane* lane) const
{
    if (numNextLane(lane)==0)
    {
        return NULL;
    }
    vector<Lane*> lanes = nextLanes(lane);
    Lane* result = lanes[0];
    double minAngle
        = abs(lane->directionVector().calcAngle
              (lanes[0]->directionVector()));
    double angle;
    for (int i=1; i<static_cast<signed int>(lanes.size()); i++)
    {
        angle
            = abs(lane->directionVector().calcAngle
                  (lanes[i]->directionVector()));
        if (angle<minAngle)
        {
            result = lanes[i];
            minAngle = angle;
        }
    }
    return result;
}

//======================================================================
int LaneBundle::numPreviousLane(const Lane* lane) const
{
    return static_cast<signed int>(previousLanes(lane).size());
}

//======================================================================
Lane* LaneBundle::previousLane(const Lane* lane, int num) const
{
    assert(0<=num && num<numPreviousLane(lane));
    return (previousLanes(lane))[num];
}

//======================================================================
Lane* LaneBundle::shortestPreviousLane(const Lane* lane) const
{
    if (numPreviousLane(lane)==0)
    {
        return NULL;
    }
    vector<Lane*> lanes = previousLanes(lane);
    Lane* result = lanes[0];
    double minLength = lanes[0]->length();
    for (int i=1; i<static_cast<signed int>(lanes.size()); i++)
    {
        if (lanes[i]->length()<minLength)
        {
            result = lanes[i];
            minLength = lanes[i]->length();
        }
    }
    return result;
}

//======================================================================
Lane* LaneBundle::mostStraightPreviousLane(const Lane* lane) const
{
    if (numPreviousLane(lane)==0)
    {
        return NULL;
    }
    vector<Lane*> lanes = previousLanes(lane);
    Lane* result = lanes[0];
    double minAngle
        = abs(lane->directionVector().calcAngle
              (lanes[0]->directionVector()));
    double angle;
    for (int i=1; i<static_cast<signed int>(lanes.size()); i++)
    {
        angle
            = abs(lane->directionVector().calcAngle
                  (lanes[i]->directionVector()));
        if (angle<minAngle)
        {
            result = lanes[i];
            minAngle = angle;
        }
    }
    return result;
}

//======================================================================
vector<Lane*> LaneBundle::lanesFromConnector(const Connector* connector) const
{
    assert(connector);
    vector<Lane*> lanes;
    map<string, Lane*, less<string> >::const_iterator itl;
    for (itl=_lanes.begin(); itl!=_lanes.end(); itl++)
    {
        if ((*itl).second->beginConnector()==connector)
        {
            lanes.push_back((*itl).second);
        }
    }
    return lanes;
}

//======================================================================
vector<Lane*> LaneBundle::lanesToConnector
(const Connector* connector) const
{
    assert(connector);
    vector<Lane*> lanes;
    map<string, Lane*, less<string> >::const_iterator itl;
    for (itl=_lanes.begin(); itl!=_lanes.end(); itl++)
    {
        if ((*itl).second->endConnector()==connector)
        {
            lanes.push_back((*itl).second);
        }
    }
    return lanes;
}

//======================================================================
void LaneBundle::renewAgentLine()
{
    map<string, Lane*, less<string> >::iterator itl;
    for (itl=_lanes.begin(); itl!=_lanes.end(); itl++)
    {
        (*itl).second->renewAgentLine();
    }
}

//======================================================================
bool LaneBundle::isHeadAgent(RoadOccupant* agent, Lane* lane) const
{
    assert(isMyLane(lane));
    bool result = false;

    // レーンの中で自分が先頭である
    if (lane->headAgent()==agent)
    {
        // 先頭レーンであればtrue
        if (isNextLaneMine(lane)==false)
        {
            result = true;
        }
        else
        {
            result = true;
            Lane* nextLane = lane;
            while (isNextLaneMine(nextLane))
            {
                nextLane = mostStraightNextLane(lane);
                if (nextLane->headAgent()!=NULL)
                {
                    result = false;
                    break;
                }
            }
        }
    }
    return result;
}

//======================================================================
list<Vehicle*>* LaneBundle::watchedVehicles()
{
    return &_watchedVehicles;
}

//======================================================================
void LaneBundle::addWatchedVehicle(Vehicle* vehicle)
{
    assert(vehicle != NULL);
    // 重複を認めない
    list<Vehicle*>::iterator itv = find(_watchedVehicles.begin(),
                                        _watchedVehicles.end(),
                                        vehicle);
    if (itv==_watchedVehicles.end())
    {
        _watchedVehicles.push_back(vehicle);
    }
    else
    {
        cerr << "_watchedVehicle has already " << vehicle->id() << endl;
    }
}

//======================================================================
void LaneBundle::eraseWatchedVehicle(Vehicle* vehicle)
{
    assert(vehicle != NULL);
    _watchedVehicles.remove(vehicle);
}
