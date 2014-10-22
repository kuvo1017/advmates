#include "VehicleShift.h"
#include "Vehicle.h"
#include "VirtualLeader.h"
#include "RoadOccupant.h"
#include "Intersection.h"
#include "Lane.h"
#include "Section.h"
#include "LaneBundle.h"
#include "GVManager.h"
#include "Conf.h"
#include <algorithm>
#include <cmath>
#include <list>
#include <cassert>

using namespace std;

//======================================================================
VehicleShift::VehicleShift()
    : _lengthTo(0), _errorTo(0), _isActive(false)
{
    _laneTo = NULL;
    _vehicle = NULL;
    _agentToNext = NULL;
    _agentToPrev = NULL;
}

//======================================================================
VehicleShift::VehicleShift(Vehicle* vehicle)
    : _lengthTo(0), _errorTo(0), _isActive(false)
{
    assert(vehicle != NULL);
    _vehicle = vehicle;
    _laneTo = NULL;
    _agentToNext = NULL;
    _agentToPrev = NULL;
}

//======================================================================
VehicleShift::~VehicleShift(){}

//======================================================================
void VehicleShift::setVehicle(Vehicle* vehicle)
{
    assert(vehicle != NULL);
    _vehicle = vehicle;
}

//======================================================================
bool VehicleShift::isActive() const
{
    return _isActive;
}

//======================================================================
bool VehicleShift::canShift(Lane* laneTo,
			    double lengthTo) const
{
    assert(_vehicle->_section);
    bool result = true;
    RoadOccupant* next = laneTo->frontAgent(lengthTo, _vehicle);
    RoadOccupant* prev = laneTo->followingAgent(lengthTo, _vehicle);
  
    double diff = 1000; // 適当な大きな値
 
    // 交差点手前10mは車線変更禁止とする．法律的には30mのはずだが...
    // MATESでは車線変更中にレーンを移ることを許さないための処置
    if(laneTo->length() - lengthTo < 10)
    {
        result = false;
    }

    if (result)
    {
        if (next != NULL)
        {
            diff = next->length() - lengthTo
                - _vehicle->bodyLength()/2
                - next->bodyLength()/2;
        }
        else
        {
            double totalLength = laneTo->length()-lengthTo;
            if (totalLength<30)
            {
                // 30m下流までは探索してみる
                LaneBundle* bundle = _vehicle->_section;
                Lane* lookupLane = laneTo;
                while (totalLength<30)
                {
                    if (!(bundle->isNextLaneMine(lookupLane)))
                    {
                        bundle = bundle->nextBundle(lookupLane);
                        if (dynamic_cast<ODNode*>(bundle))
                        {
                            break;
                        }
                    }

                    lookupLane = bundle->mostStraightNextLane(lookupLane);
                    next = lookupLane->tailAgent(_vehicle);
                    if (next != NULL)
                    {
                        diff = next->length() + totalLength
                            - _vehicle->bodyLength()/2
                            - next->bodyLength()/2;
                        break;
                    }
                    else
                    {
                        totalLength += lookupLane->length();
                    }
                }
            }
        }
        if(next != NULL
           && (diff<(_vehicle->velocity() - next->velocity())*2000
               || diff<5))
        {
            // 減速を試みる?
            result = false;
        }
    }

    if (result)
    {
        diff = 1000; // 適当な大きな値
        if (prev!=NULL)
        {
            diff = lengthTo - prev->length()
                - _vehicle->bodyLength()/2
                - prev->bodyLength()/2;
        }
        else
        {
            double totalLength = lengthTo;
            if (totalLength<30)
            {
                // 30m上流流までは探索してみる
                LaneBundle* bundle = _vehicle->_section;
                Lane* lookupLane = laneTo;
                while (totalLength<30)
                {
                    if (!(bundle->isPreviousLaneMine(lookupLane)))
                    {
                        bundle = bundle->previousBundle(lookupLane);
                        if (dynamic_cast<ODNode*>(bundle))
                        {
                            break;
                        }
                    }
                    lookupLane = bundle->mostStraightPreviousLane(lookupLane);
                    prev = lookupLane->headAgent(_vehicle);
                    if (prev != NULL)
                    {
                        diff = lookupLane->length() - prev->length() + totalLength
                            - _vehicle->bodyLength()/2
                            - prev->bodyLength()/2;
                        break;
                    }
                    else
                    {
                        totalLength += lookupLane->length();
                    }
                }
            }
        }
        if (prev != NULL
            && (diff<(prev->velocity()-_vehicle->velocity())*2000
                || diff<5))
        {
            // 加速を試みる?
            result = false;
        }
    }

    // 次の交差点に到達するまでに車線変更を完了しなければならない
    // 交差点形状詳細指定をすると上下でレーン幅が変わる可能性がある、現状は一定の値にしている
    // 隣のレーンに移るまでに必要な時間[msec]
    double timeToSideLane
        = _vehicle->_section->laneWidth() / 
        (GVManager::instance().getNumeric("ERROR_VELOCITY")/60.0/60.0);

    // 次の交差点までの間隔が狭い場合は車線変更できない
    // 速度を維持したまま車線変更できるか判定
    // 不可能な場合は減速し、次のステップで再判定
    if(result &&
       _vehicle->_section->lengthToNext(laneTo, lengthTo)
       < _vehicle->_velocity*timeToSideLane)
    {
        result = false;
#ifndef VL_DEBUG
        VirtualLeader* leader
            = new VirtualLeader(_vehicle->_section->lengthToNext(laneTo, lengthTo),
                                10.0/60/60);
#else
        VirtualLeader* leader
            = new VirtualLeader(_vehicle->_section->lengthToNext(laneTo, lengthTo),
                                10.0/60/60,
                                "SHIFTDEC0:"+_vehicle->_section->id());
#endif
        _vehicle->_leaders.push_back(leader);
    }

    return result;
}

//======================================================================
void VehicleShift::beginShift(Lane* laneTo, double lengthTo)
{
    assert(_vehicle->_section);
    _laneTo = laneTo;
    _lengthTo = lengthTo;
  
    _agentToNext = laneTo->frontAgent(lengthTo, _vehicle);
    _agentToPrev = laneTo->followingAgent(lengthTo, _vehicle);
  
    _vehicle->notify();
    _isActive = true;
}

//======================================================================
void VehicleShift::endShift()
{
    assert(_isActive == true);
    Lane* originalLane = _vehicle->_lane;

    originalLane->extractAfterEraseAgent(_vehicle);
    _laneTo->extractAfterAddAgent(_vehicle, _agentToNext);

    if(_vehicle->_intersection)
    {
        _vehicle->setLane(_vehicle->_intersection,
                          _laneTo,
                          _lengthTo);
    }
    else
    {
        _vehicle->setLane(_vehicle->_section,
                          _laneTo,
                          _lengthTo);
    }
    _vehicle->_length = _lengthTo;
    _vehicle->_error = 0;

    _laneTo = NULL;
    _lengthTo = 0;

    _agentToNext = NULL;
    _agentToPrev = NULL;
  
    _vehicle->unnotify();
    _isActive = false;
}

//======================================================================
bool VehicleShift::proceedShift()
{
    bool result = false;
    assert(_isActive == true);
    if(_vehicle->_intersection)
    {
    }
    else
    {
        int isLeft;
        _lengthTo = _vehicle->_section->lengthOnSideLane(_vehicle->_lane,
                                                         _laneTo,
                                                         _vehicle->_length,
                                                         &isLeft);
    }
    _agentToNext = _laneTo->frontAgent(_lengthTo, _vehicle);
    _agentToPrev = _laneTo->followingAgent(_lengthTo, _vehicle);

    if (_agentToNext!=NULL)
    {
        double diff = _agentToNext->length()-_lengthTo;
        // ここは Vehicle 以外のエージェントも考慮している
        Vehicle* frontVehicle = dynamic_cast<Vehicle*>(_agentToNext);
        VirtualLeader* leader;
        if (frontVehicle!=NULL)
        {
#ifndef VL_DEBUG
            leader = new VirtualLeader(diff, _agentToNext->velocity());
#else
            leader = new VirtualLeader(diff, _agentToNext->velocity(),
                                       "SHIFTFRONT:"+frontVehicle->id());
#endif
        }
        else
        {
#ifndef VL_DEBUG
            leader = new VirtualLeader(diff, 0);
#else
            leader = new VirtualLeader(diff, 0, "SHIFTPDS");
#endif
        }
        _vehicle->_leaders.push_back(leader);
    }

    // 次の交差点に到達するまでに車線変更を完了しなければならない
    // 交差点形状詳細指定をすると上下でレーン幅が変わる可能性がある、現状は一定の値にしている
    // 隣のレーンに移るまでに必要な時間[msec]
    double timeToSideLane
        = (_vehicle->_section->laneWidth() - _vehicle->_error)
        / (GVManager::instance().getNumeric("ERROR_VELOCITY")/60.0/60.0);
    // 速すぎる場合は減速
    if(_vehicle->_section->lengthToNext(_laneTo, _lengthTo)
       < _vehicle->_velocity*timeToSideLane)
    {
#ifndef VL_DEBUG
        VirtualLeader* leader
            = new VirtualLeader(_vehicle->_section->lengthToNext(_laneTo, _lengthTo),
                                10.0/60/60);
#else
        VirtualLeader* leader
            = new VirtualLeader(_vehicle->_section->lengthToNext(_laneTo, _lengthTo),
                                10.0/60/60,
                                "SHIFTDEC1:"+_vehicle->_section->id());
#endif
        _vehicle->_leaders.push_back(leader);
    }


    // ここに車線変更の終了条件をいれる
    if(abs(_vehicle->_error) >= _vehicle->_section->laneWidth())
    {
        result = true;
    }
    return result;
}

//======================================================================
void VehicleShift::abortShift()
{
    assert(_isActive);
    _isActive = false;
    _vehicle->_error = 0;

    _laneTo = NULL;
    _lengthTo = 0;
  
    _agentToNext = NULL;
    _agentToPrev = NULL;
  
    _vehicle->unnotify();
}

//======================================================================
void VehicleShift::searchInterruption() const
{
    assert(_vehicle->_section!=NULL);

    // 車線変更中の車両を取得する
    list<Vehicle*>* notifyVehicles
        = _vehicle->_section->watchedVehicles();
    list<Vehicle*>::const_iterator itv;
    Vehicle* notifyVehicle = NULL;

    for(itv = notifyVehicles->begin();
        itv != notifyVehicles->end();
        itv++)
    {
        VehicleShift anotherVehicleShift = (*itv)->shiftLane();

        if (anotherVehicleShift.isActive()
            && anotherVehicleShift.laneTo() == _vehicle->_lane
            && anotherVehicleShift.lengthTo() > _vehicle->_length)
        {
            if(notifyVehicle == NULL
               || notifyVehicle->length() > (*itv)->length())
            {
                notifyVehicle = (*itv);
            }
        }
    }

    if(notifyVehicle!=NULL)
    {
        double diff = notifyVehicle->shiftLane().lengthTo() - _vehicle->length();
#ifndef VL_DEBUG
        VirtualLeader* leader = new VirtualLeader(diff, notifyVehicle->velocity());
#else
        VirtualLeader* leader = new VirtualLeader(diff, notifyVehicle->velocity(),
                                                  "INTERRUPT:"+notifyVehicle->id()); 
#endif
        _vehicle->_leaders.push_back(leader);
    }
}

//======================================================================
double VehicleShift::activeErrorVelocity() const
{
    double result = 0;
    if(_isActive)
    {
        int direction = 0;
        _vehicle->_section->lengthOnSideLane(_vehicle->_lane,
                                             _laneTo,
                                             _vehicle->_length,
                                             &direction);
        result
            = GVManager::instance().getNumeric("ERROR_VELOCITY")/60.0/60.0
            * direction;
    }
    return result;
}

//======================================================================
Lane* VehicleShift::laneTo() const
{
    return _laneTo;
}

//======================================================================
double VehicleShift::lengthTo() const
{
    return _lengthTo;
}
//======================================================================
double VehicleShift::errorTo() const
{
    return _errorTo;
}

//======================================================================
const RoadOccupant* VehicleShift::agentToNext() const
{
    return _agentToNext;
}

