#include <iostream>
#include <cassert>
#include <algorithm>
#include "Lane.h"
#include "AmuVector.h"
#include "RoadOccupant.h"
#include "Conf.h"
#include "Vehicle.h"

using namespace std;

//######################################################################
/// エージェントのソートに用いるクラス
class AgentLess
{
public:
    bool operator()(const RoadOccupant* rl, const RoadOccupant* rr) const
    {
        return (rl->length() < rr->length());
    }
};

//######################################################################
Lane::Lane(const string& id, const Connector* ptBegin,
           const Connector* ptEnd)
    :_lineSegment(AmuLineSegment(ptBegin->point(), ptEnd->point()))
{
    assert(ptBegin && ptEnd);

    _id = id;
    _beginConnector = ptBegin;
    _endConnector = ptEnd;
    _lastArrivalTime = 0;

//勾配の設定
    double xx, zz;
    xx = sqrt( (_endConnector->x() - _beginConnector->x())
               * (_endConnector->x() - _beginConnector->x())
               + (_endConnector->y() - _beginConnector->y())
               * (_endConnector->y() - _beginConnector->y()) );
    zz = _endConnector->z() - _beginConnector->z();
    _gradient = zz / xx * 100;
}

//======================================================================
Lane::~Lane(){}

//======================================================================
void Lane::setId(const string& id)
{
    _id = id;
}

//======================================================================
const string& Lane::id() const
{
    return _id;
}

//======================================================================
const Connector* Lane::beginConnector() const
{
    return _beginConnector;
}

//======================================================================
const Connector* Lane::endConnector() const
{
    return _endConnector;
}

//======================================================================
double Lane::length() const
{
    return _lineSegment.length();
}

//======================================================================
const AmuLineSegment Lane::lineSegment() const
{
    return _lineSegment;
}

//======================================================================
const AmuVector Lane::directionVector() const
{
    return _lineSegment.directionVector();
}

//======================================================================
double Lane::speedLimit() const
{
    return _speedLimit;
}

//======================================================================
void Lane::setSpeedLimit(double limit)
{
    _speedLimit = limit;
}

//======================================================================
ulint Lane::lastArrivalTime() const
{
    return _lastArrivalTime;
}

//======================================================================
void Lane::setLastArrivalTime(ulint arrivalTime)
{
    _lastArrivalTime = arrivalTime;
}

//======================================================================
bool Lane::createIntersectionPoint(const AmuLineSegment& crSLine,
				   AmuPoint* result_point) const
{
    bool result = false;
    result = _lineSegment.createIntersectionPoint(crSLine,
                                                  result_point);
    return result;
}

//======================================================================
AmuPoint Lane::createInteriorPoint(double d0, double d1) const
{
    return _lineSegment.createInteriorPoint(d0, d1);
}

//======================================================================
const AmuPoint Lane::calcNearestPoint(AmuPoint point) const
{
    return _lineSegment.calcNearestPoint(point);
}

//======================================================================
vector<RoadOccupant*>* Lane::agents()
{
    return &_agents;
}

//======================================================================
RoadOccupant* Lane::headAgent()
{
    // _agentsはlengthの小さな(始点に近い)ものから順に並んでいる
    RoadOccupant* head = NULL;
    if (!_agents.empty())
    {
        head = _agents.back();
    }
    return head;
}

//======================================================================
RoadOccupant* Lane::headAgent(Vehicle* sightVehicle)
{
    return headAgent();
}

//======================================================================
Vehicle* Lane::headVehicle()
{
    Vehicle* head = NULL;
    if (!_agents.empty())
    {
        vector<RoadOccupant*>::reverse_iterator ita;
        for (ita=_agents.rbegin(); ita!=_agents.rend(); ita++)
        {
            try
            {
                head = dynamic_cast<Vehicle*>(*ita);
            }
            catch(...)
            {
                cout << "dynamic_cast error" << (*ita)->id() << endl;
                assert(0);
            }
            if (head!=NULL)
            {
                break;
            }
        }
    }
    return head;
}

//======================================================================
RoadOccupant* Lane::tailAgent()
{
    // _agentsはlengthの小さな(始点に近い)ものから順に並んでいる
    if (!_agents.empty())
    {
        return _agents.front();
    }
    return NULL;
}

//======================================================================
RoadOccupant* Lane::tailAgent(Vehicle* sightVehicle)
{
    return tailAgent();
}

//======================================================================
Vehicle* Lane::tailVehicle()
{
    // _agentsはlengthの小さな(始点に近い)ものから順に並んでいる
    Vehicle* tail = NULL;
    if (!_agents.empty())
    {
        vector<RoadOccupant*>::iterator ita;
        for (ita=_agents.begin(); ita!=_agents.end(); ita++)
        {
            try
            {
                tail = dynamic_cast<Vehicle*>(*ita);
            }
            catch(...)
            {
                cout << "dynamic_cast error" << (*ita)->id() << endl;
                exit(EXIT_FAILURE);
            }
            if (tail!=NULL)
            {
                break;
            }
        }
    }
    return tail;
}

//======================================================================
RoadOccupant* Lane::tailAgentStrict(Vehicle* sightVehicle)
{
    return foreAgentStrict(tailAgent());
}

//======================================================================
RoadOccupant* Lane::frontAgent(RoadOccupant* agent)
{
    RoadOccupant* front = NULL;
    std::vector<RoadOccupant*>::iterator ita = find(_agents.begin(),
                                               _agents.end(),
                                               agent);
    if (ita != _agents.end())
    {
        ita++;
        if (ita != _agents.end())
        {
            front = *ita;
        }
    }
    return front;
}

//======================================================================
RoadOccupant* Lane::frontAgent(RoadOccupant* agent, Vehicle* sightVehicle)
{
    return frontAgent(agent);
}

//======================================================================
Vehicle* Lane::frontVehicle(RoadOccupant* agent)
{
    RoadOccupant* front = frontAgent(agent);
    while (front != NULL && dynamic_cast<Vehicle*>(front) == NULL)
    {
        front = frontAgent(front);
    }
    return dynamic_cast<Vehicle*>(front);
}

//======================================================================
RoadOccupant* Lane::frontAgent(double length)
{
    RoadOccupant* front = NULL;
    vector<RoadOccupant*>::const_iterator ita;
    for (ita = _agents.begin(); ita!=_agents.end(); ita++)
    {
        if ((*ita)->length()>= length)
        {
            front = (*ita);
            break;
        }
    }
    return front;
}

//======================================================================
RoadOccupant* Lane::frontAgent(double length, Vehicle* sightVehicle)
{
    return frontAgent(length);
}

//======================================================================
Vehicle* Lane::frontVehicle(double length)
{
    RoadOccupant* front = frontAgent(length);
    while (front != NULL && dynamic_cast<Vehicle*>(front) == NULL)
    {
        front = frontAgent(front);
    }
    return dynamic_cast<Vehicle*>(front);
}

//======================================================================
RoadOccupant* Lane::frontAgentStrict(double length, Vehicle* sightVehicle)
{
#ifndef USE_ADDIN
    return foreAgentStrict(frontAgent(length));
#else
    return foreAgentStrict(frontAgent(length, sightVehicle));
#endif
}

//======================================================================
RoadOccupant* Lane::followingAgent(RoadOccupant* agent)
{
    RoadOccupant* follower = NULL;
    vector<RoadOccupant*>::const_iterator ita = find(_agents.begin(),
                                                     _agents.end(),
                                                     agent);
    if (ita!=_agents.begin()
        && ita!=_agents.end())
    {
        ita--;
        follower = *ita;
    }
    return follower;
}

//======================================================================
RoadOccupant* Lane::followingAgent(double length, Vehicle* sightVehicle)
{
    return followingAgent(length);
}

//======================================================================
Vehicle* Lane::followingVehicle(RoadOccupant* agent)
{
    RoadOccupant* follower = followingAgent(agent);
    while (follower != NULL && dynamic_cast<Vehicle*>(follower) == NULL)
    {
        follower = followingAgent(follower);
    }
    return dynamic_cast<Vehicle*>(follower);
}

//======================================================================
RoadOccupant* Lane::followingAgent(double length)
{
    RoadOccupant* follower = NULL;
    vector<RoadOccupant*>::const_iterator ita;
    for (ita = _agents.begin(); ita!=_agents.end(); ita++)
    {
        if ((*ita)->length()<length)
        {
            follower = *ita;
        }
        else
        {
            break;
        }
    }

    return follower;
}

//======================================================================
Vehicle* Lane::followingVehicle(double length)
{
    RoadOccupant* follower = followingAgent(length);
    while (follower != NULL && dynamic_cast<Vehicle*>(follower) == NULL)
    {
        follower = followingAgent(follower);
    }
    return dynamic_cast<Vehicle*>(follower);
}

//======================================================================
RoadOccupant* Lane::foreAgentStrict(RoadOccupant* agent)
{
    if (agent != NULL && dynamic_cast<Vehicle*>(agent) == NULL)
    {
        Vehicle* nextVehicle = frontVehicle(agent);
        if ((nextVehicle != NULL)
            && (agent->length() - agent->bodyLength() / 2
                > nextVehicle->length() - nextVehicle->bodyLength() / 2))
        {
            return nextVehicle;
        }
    }
    return agent;
}

//======================================================================
bool Lane::putAgent(RoadOccupant* agent)
{
    bool flag = false;
#ifdef _OPENMP
#pragma omp critical (Lane)
    {
#endif
        vector<RoadOccupant*>::iterator ita = find(_tmpAgents.begin(),
                                                   _tmpAgents.end(),
                                                   agent);
        if (ita == _tmpAgents.end())
        {
            _tmpAgents.push_back(agent);
            flag = true;
        }
        else
        {
            assert(0);
        }
#ifdef _OPENMP
    }
#endif
    return flag;
}

//======================================================================
void Lane::renewAgentLine()
{
    // 2つのvectorの入れ替え
    _agents.swap(_tmpAgents);
    // ソートする
    sort(_agents.begin(), _agents.end(), AgentLess());
    // tmpを空に
    _tmpAgents.clear();
}

//======================================================================
void Lane::extractAfterEraseAgent(RoadOccupant* agent)
{
    vector<RoadOccupant*>::iterator eraseTarget = find(_agents.begin(),
                                                       _agents.end(),
                                                       agent);
    assert(eraseTarget!=_agents.end());
    _agents.erase(eraseTarget);
}

//======================================================================
void Lane::extractAfterAddAgent(RoadOccupant* agent,
				RoadOccupant* front)
{
    /*
     * _agentsはlengthの小さな(始点に近い)ものから順に並んでいる
     * frontがNULL(=先頭に追加する)場合には_agentsの最後に追加
     * frontがNULLでなければ，frontの「前（始点に近い側）」に追加
     */
    if (front==NULL)
    {
        _agents.push_back(agent);
    }
    else
    {
        vector<RoadOccupant*>::iterator addTarget = find(_agents.begin(),
                                                         _agents.end(),
                                                         front);
        assert(addTarget!=_agents.end());
        _agents.insert(addTarget, agent);
    }
}

//======================================================================
double Lane::gradient() const
{
    return _gradient;
}

//======================================================================
double Lane::averageVel() const
{
    double vel = 0.0;
    int numVehicles = 0;

    for (unsigned int i=0; i<_tmpAgents.size(); i++)
    {
        if (dynamic_cast<Vehicle*>(_tmpAgents[i])!=NULL)
        {
            numVehicles++;
            vel += _tmpAgents[i]->velocity();
        }
    }

    // 車両がいない場合には制限速度を平均速度とする
    if (numVehicles==0)
    {
        vel = _speedLimit/60.0/60.0;
    }
    else
    {
        vel /= numVehicles;
    }
    return vel;
}
