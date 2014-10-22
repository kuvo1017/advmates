#include "ODNode.h"
#include "LaneBundle.h"
#include "Intersection.h"
#include "Section.h"
#include "Lane.h"
#include "RoadMap.h"
#include "Vehicle.h"
#include "Route.h"
#include "ObjManager.h"
#include "GenerateVehicleIO.h"
#include "VehicleIO.h"
#include "VehicleShift.h"
#include "GVManager.h"
#include "Random.h"
#include <iostream>
#include <string>
#include <vector>
#include <deque>
#include <list>
#include <algorithm>
#include <cassert>

using namespace std;

//======================================================================
ODNode::ODNode(const string& id, const string& type)
    : Intersection(id, type)
{
    _lastGenTime = 0;
    _nodeGvd.clear();
    _isCount = false;
}

//======================================================================
ODNode::~ODNode()
{
    // ODNode特有の変数をここでdelete
    for (int i=0;
         i<static_cast<signed int>(_waitingVehicles.size());
         i++)
    {
        delete _waitingVehicles[i];
    }
    _waitingVehicles.clear();
}

//======================================================================
bool ODNode::hasWaitingVehicles() const
{
    if (!_waitingVehicles.empty())
    {
        return true;
    }
    else
    {
        return false;
    }
}

//======================================================================
void ODNode::addWaitingVehicle(Vehicle* vehicle)
{
    assert(vehicle);
    _waitingVehicles.push_back(vehicle);
}

//======================================================================
void ODNode::addWaitingVehicleFront(Vehicle* vehicle)
{
    assert(vehicle);
    _waitingVehicles.push_front(vehicle);
}

//======================================================================
#ifndef _OPENMP
void ODNode::pushVehicleToReal(RoadMap* roadMap, bool isWrite)
#else
void ODNode::pushVehicleToReal(RoadMap* roadMap, bool isWrite, int genIntensiveStep)
#endif
{
    assert(!_waitingVehicles.empty());
    
    Section* section = _incSections[0];
    vector<Lane*> lanes = section->lanesFrom(this);

    // 他車両の車線変更先となっている車線を取得する
    // 発生点付近で車線変更が行われている場合は車両は発生できない
    vector<Lane*> shiftTargetLanes;
    list<Vehicle*>* notifyVehicles = section->watchedVehicles();

    list<Vehicle*>::iterator itv;
    for (itv = notifyVehicles->begin();
         itv != notifyVehicles->end();
         itv++)
    {
        VehicleShift anotherVehicleShift = (*itv)->shiftLane();
        if (anotherVehicleShift.isActive() &&
            anotherVehicleShift.lengthTo() < 25)
        {
            shiftTargetLanes.push_back(const_cast<Lane*>(anotherVehicleShift.laneTo()));
        }
    }
    
    // 十分な空きのあるレーンを取得
    deque<Lane*> possibleLanes;
    for (int i=0;
         i<static_cast<signed int>(lanes.size());
         i++)
    {

        vector<Lane*>::iterator itl = find(shiftTargetLanes.begin(),
                                           shiftTargetLanes.end(),
                                           lanes[i]);
        if (itl!=shiftTargetLanes.end()) continue;
   
        if (lanes[i]->tailAgent()==NULL)
        {
            possibleLanes.push_back(lanes[i]);
        }
#ifdef GENERATE_VELOCITY_0
        else if (lanes[i]->tailAgent()->length()
                 - lanes[i]->tailAgent()->bodyLength()/2
                 - _waitingVehicles.front()->bodyLength()
                 > 1.0)
        {
            possibleLanes.push_back(lanes[i]);
        }
#else
        else if (lanes[i]->tailAgent()->length()
                 - lanes[i]->tailAgent()->bodyLength()/2
                 - _waitingVehicles.front()->bodyLength()
                 > 1.38+lanes[i]->speedLimit()/60.0/60.0*740)
        {
            possibleLanes.push_back(lanes[i]);
        }
#endif // GENERATE_VELOCITY_0
    }

    vector<Vehicle*> skipVehicles;
    if (!possibleLanes.empty())
    {
        // 空きレーンがある場合に限り車両を発生させる
        vector<int> order = Random::randomOrder(possibleLanes.size());

        while (!_waitingVehicles.empty() && !order.empty())
        {
            Vehicle* tmpVehicle = _waitingVehicles.front();
            tmpVehicle->setStartTime(TimeManager::time());
#ifdef _OPENMP
            if (tmpVehicle->genIntensiveStep() > genIntensiveStep)
            {
                break;
            }
#endif
            _waitingVehicles.pop_front();

            Lane* generateLane = NULL;
            generateLane = possibleLanes[order.back()];
            order.pop_back();
            
            if (generateLane == NULL)
            {
                skipVehicles.push_back(tmpVehicle);
            }
            else
            {
                tmpVehicle->addToSection(roadMap, section, generateLane,
                                         tmpVehicle->bodyLength()/2);
                const_cast<Route*>(tmpVehicle->route())->setLastPassedIntersection(this);

                bool result = ObjManager::addVehicleToReal(tmpVehicle);
                assert(result);

                if (isWrite)
                {
                    // addVehicleToRealの後でなければIDが決まらない
                    VehicleIO::instance().writeVehicleStaticData(tmpVehicle);
                }

                if (_isCount)
                {
                    GeneratedVehicleData gvd;
                    gvd.vehicle = tmpVehicle;
                    gvd.lane = generateLane;
                    gvd.headway = TimeManager::time()-_lastGenTime;
                    _nodeGvd.push_back(gvd);
                }
                _lastGenTime = TimeManager::time();
            }
        }
    }

    for (unsigned int i = 0; i < skipVehicles.size(); i++)
    {
        _waitingVehicles.push_front(skipVehicles[i]);
    }

    // 車両発生情報の出力
    if (_isCount && !_nodeGvd.empty())
    {
        if (isWrite)
        {
            GenerateVehicleIO::writeGeneratedVehicleData(this, &_nodeGvd);
        }
        _nodeGvd.clear();
    }
}

//======================================================================
void ODNode::deleteAgent(bool isWrite)
{
    map<string, Lane*, less<string> >::iterator itl;
    
    for (itl=_lanes.begin(); itl!=_lanes.end(); itl++)
    {
        vector<RoadOccupant*>* agentsOfLane = (*itl).second->agents();
        for (int i=0;
             i<static_cast<signed int>(agentsOfLane->size());
             i++)
        {
            if (dynamic_cast<Vehicle*>((*agentsOfLane)[i]))
            {
                if (isWrite)
                {
                    // 消去する前にトリップ長の出力
                    VehicleIO::instance().writeVehicleDistanceData
                        (dynamic_cast<Vehicle*>((*agentsOfLane)[i]));
                }

                ObjManager::deleteVehicle(dynamic_cast<Vehicle*>((*agentsOfLane)[i]));
            }
        }
    }
}

//======================================================================
void ODNode::clearGVD(GeneratedVehicleData* gvd)
{
    gvd->vehicle = NULL;
    gvd->lane    = NULL;
    gvd->headway = 0;
}

//======================================================================
bool ODNode::isCount() const
{
    return _isCount;
}

//======================================================================
void ODNode::setCountState(const bool isCount)
{
    _isCount = isCount;
}
