#include "Section.h"
#include "RoadMap.h"
#include "Intersection.h"
#include "Lane.h"
#include "RoadEntity.h"
#include "RoadOccupant.h"
#include "Vehicle.h"
#include "GVManager.h"
#include "AmuLineSegment.h"
#include "Conf.h"
#include <cassert>

using namespace std;

//======================================================================
Section::Section(const string& id, 
		 Intersection* first, 
		 Intersection* second,
                 RoadMap* parent) : LaneBundle(id, parent)
{
    _adjInter[0] = first;
    _adjInter[1] = second;
    _hasStructInfo = false;
}

//======================================================================
Section::~Section()
{
    // _entitiesの消去
    ITRMAPENT ite;
    for (ite=_entities.begin(); ite!=_entities.end(); ite++)
    {
        delete (*ite).second;
        (*ite).second = NULL;
    }

    // _lanesの消去
    ITRMAPLAN itl;
    for (itl=_lanes.begin(); itl!=_lanes.end(); itl++)
    {
        delete (*itl).second;
        (*itl).second = NULL;
    }

    _adjInter[0] = 0;
    _adjInter[1] = 0;
}

//======================================================================
const AmuPoint Section::center() const
{
    return _center;
}

//======================================================================
double Section::length() const
{
    assert(_vertices.size()==4);
    return ( AmuLineSegment(_vertices[1],_vertices[2]).length()
             + AmuLineSegment(_vertices[3],_vertices[0]).length() )/2;
}

//======================================================================
double Section::width() const
{
    return (_numIn[0]+_numIn[1]+_numOut[0]+_numOut[1])/2.0;
}

//======================================================================
double Section::upWidth() const
{
    return _numIn[0];
}

//======================================================================
double Section::downWidth() const
{
    return _numIn[1];
}

//======================================================================
LaneBundle* Section::nextBundle(Lane* lane) const
{
    return nextIntersection(lane);
}

//======================================================================
LaneBundle* Section::previousBundle(Lane* lane) const
{
    return previousIntersection(lane);
}

//======================================================================
Intersection* Section::intersection(bool isUp) const
{
    /*
     * _adjInter[0]->_adjInter[1]が上り方向
     * isUpがtrueであれば_adjInter[1]を
     * isUpがfalseであれば_adjInter[0]を返す
     */
    if (isUp)
    {
        return _adjInter[1];
    }
    else
    {
        return _adjInter[0];
    }
}

//======================================================================
Intersection* Section::nextIntersection(Lane* lane) const
{
    if (isNextLaneMine(lane)) 
    {
        return NULL;
    }
    else
    {
        return intersection(isUp(lane));
    }
}

//======================================================================
Intersection* Section::previousIntersection(Lane* lane) const
{
    if (isPreviousLaneMine(lane)) 
    {
        return NULL;
    }
    else
    {
        return intersection(!isUp(lane));
    }
}

//======================================================================
vector<Lane*> Section::nextLanes(const Lane* lane) const
{
    vector<Lane*> result_lanes = lanesFromConnector(lane->endConnector());
    if (result_lanes.empty())
    {
        // 次のレーンが単路にない場合
        Intersection* nextInter = intersection(isUp(lane));
        result_lanes = nextInter->lanesFromConnector(lane->endConnector());
    }
    if (result_lanes.empty())
    {
        cerr << "no next lane error at " << _id << endl;
        exit(1);
    }
    return result_lanes;
}

//======================================================================
vector<Lane*> Section::previousLanes(const Lane* lane) const
{
    vector<Lane*> result_lanes
        = lanesToConnector(lane->beginConnector());
    if (result_lanes.empty())
    {
        // 前のレーンが単路にない場合
        Intersection* prevInter = intersection(!isUp(lane));
        result_lanes
            = prevInter->lanesToConnector(lane->beginConnector());
    }
    if (result_lanes.empty())
    {
        cerr << "no previous lane error at " << _id << endl;
        exit(1);
    }
    return result_lanes;
}

//======================================================================
bool Section::isReachable(const Lane* lane, int dir) const
{
    bool result = false;
    vector<Lane*> lanes = nextLanes(lane);
    if (isNextLaneMine(lane))
    {
        vector<Lane*> lanes = nextLanes(lane);
        // 次のレーンも単路内なら引き続き検索
        for (unsigned int i=0; i<lanes.size() && !result; i++)
        {
            result = isReachable(lanes[i], dir);
        }
    }
    else
    {
        // 交差点なら交差点のisReachableを呼び出す
        Intersection* inter = intersection(isUp(lane));
        for (unsigned int i=0; i<lanes.size() && !result; i++)
        {
            result = inter->isReachable(lanes[i], dir);
        }
    }
    return result;
}

//======================================================================
RoadEntity* Section::pairedEntity(RoadEntity* entity,
                                  int edge) const
{
    assert(0<=edge && edge<entity->numVertices());
    AmuLineSegment commonEdge = entity->edge(edge);

    // 同一の単路内から検索
    CITRMAPENT ite;
    for (ite=_entities.begin(); ite!=_entities.end(); ite++)
    {
        if ((*ite).second==entity)
        {
            continue;
        }
        for (int i=0;
             i<static_cast<signed int>((*ite).second->numVertices());
             i++)
        {
            AmuLineSegment line = (*ite).second->edge(i);
            // 対象の辺と共通した始点終点を持つサブセクションを探して返す
            if (( commonEdge.pointBegin()==line.pointBegin()
                  && commonEdge.pointEnd()==line.pointEnd() )
                || ( commonEdge.pointBegin()==line.pointEnd()
                     && commonEdge.pointEnd()==line.pointBegin()) )
            {
                return (*ite).second;
            }
        }
    }
    for (int j=0; j<2; j++)
    {
        for (ite=_adjInter[j]->entities()->begin();
             ite!=_adjInter[j]->entities()->end();
             ite++)
        {
            for (int i=0;
                 i<static_cast<signed int>((*ite).second->numVertices());
                 i++)
            {
                AmuLineSegment line = (*ite).second->edge(i);
                // 対象の辺と共通した始点終点を持つサブセクションを返す
                if (( commonEdge.pointBegin()==line.pointBegin()
                      && commonEdge.pointEnd()==line.pointEnd() )
                    || ( commonEdge.pointBegin()==line.pointEnd()
                         && commonEdge.pointEnd()==line.pointBegin()) )
                {
                    return (*ite).second;
                }
            }
        }
    }
    return NULL;
}

//======================================================================
vector<Lane*> Section::lanesFrom(const Intersection* inter) const
{
    assert(inter==_adjInter[0] || inter==_adjInter[1]);
    vector<Lane*> result_lanes;
    vector<const Connector*> connectors
        = inter->border(inter->direction(this))->outPoints();

    for (int i=0; i<static_cast<signed int>(connectors.size()); i++)
    {
        vector<Lane*>lanes = lanesFromConnector(connectors[i]);
        for (int j=0; j<static_cast<signed int>(lanes.size()); j++)
        {
            result_lanes.push_back(lanes[j]);
        }
    }
    return result_lanes;
}

//======================================================================
vector<Lane*> Section::lanesTo(const Intersection* inter) const
{
    assert(inter==_adjInter[0] || inter==_adjInter[1]);
    vector<Lane*> result_lanes;
    vector<const Connector*> connectors
        = inter->border(inter->direction(this))->inPoints();

    for (int i=0; i<static_cast<signed int>(connectors.size()); i++)
    {
        vector<Lane*>lanes = lanesToConnector(connectors[i]);
        for (int j=0; j<static_cast<signed int>(lanes.size()); j++)
        {
            result_lanes.push_back(lanes[j]);
        }
    }
    return result_lanes;
}

//======================================================================
bool Section::isUp(const Lane* lane) const
{
    //IDが"00"から始まる
    int idBorder = atoi(lane->id().c_str())/1000000;
    if (idBorder==0)
    {
        return true;
    } 
    //IDが"01"から始まる
    else if (idBorder==1)
    {
        return false;
    }
    //それ以外は前のレーンで判定する
    else
    {
        assert(lanesToConnector(lane->beginConnector()).size()!=0);
        return isUp(lanesToConnector(lane->beginConnector())[0]);
    }
}

//======================================================================
int Section::_numAgents(double start, double len, bool up) const
{
    int result = -1;
    if (start < 0.1) start = 0;
    if (len > length()) len = length();
    if (start < length())
    {
        result = 0;
        vector<RoadOccupant*>* agents;
        vector<RoadOccupant*>::const_iterator ita;
        CITRMAPLAN itl;
        for (itl=_lanes.begin(); itl!=_lanes.end(); itl++)
        {
            if (isUp((*itl).second) == up) {
                agents = (*itl).second->agents();
                for (ita=agents->begin(); ita!=agents->end(); ita++)
                {
                    // 統計情報に含めるかどうか判断
                    // 例えば路面電車は統計に含めない
                    if ((*ita)->totalLength() >= start &&
                        (*ita)->totalLength() < start+len)
                    {
                        result++;
                    }
                }
            }
        }
    }
    return result;
}

//======================================================================
int Section::numUpAgents(double start, double len) const
{
    return _numAgents(start, len, true);
}

//======================================================================
int Section::numDownAgents(double start, double len) const
{
    return _numAgents(start, len, false);
}

//======================================================================
double Section::averageUpVel() const
{
    double vel = 0.0;
    int numVehicles = 0;

    // 上り方向に進むレーンを抽出
    CITRMAPLAN itl;
    for (itl=_lanes.begin(); itl!=_lanes.end(); itl++)
    {
        if (isUp((*itl).second))
        {
            vector<RoadOccupant*>* as = (*itl).second->agents();
            for (int i=0; i<static_cast<signed int>(as->size()); i++)
            {
                if (dynamic_cast<Vehicle*>((*as)[i])!=NULL)
                {
                    vel += (*as)[i]->velocity();
                    numVehicles++;
                }
            }
        }
    }
    if (numVehicles!=0)
    {
        vel /= numVehicles;
    }

    return vel;
}

//======================================================================
double Section::averageDownVel() const
{
    double vel = 0.0;
    int numVehicles = 0;

    // 下り方向に進むレーンを抽出
    CITRMAPLAN itl;
    for (itl=_lanes.begin(); itl!=_lanes.end(); itl++)
    {
        if (!isUp((*itl).second))
        {
            vector<RoadOccupant*>* as = (*itl).second->agents();
            for (int i=0; i<static_cast<signed int>(as->size()); i++)
            {
                if (dynamic_cast<Vehicle*>((*as)[i])!=NULL)
                {
                    vel += (*as)[i]->velocity();
                    numVehicles++;
                }
            }
        }
    }
    if (numVehicles!=0)
    {
        vel /= numVehicles;
    }

    return vel;
}

//======================================================================
double Section::time(Intersection* in0, Intersection* in1) const
{
    if (in0==_adjInter[0] && in1==_adjInter[1])
    {
        return upTime();
    }
    else if (in0==_adjInter[1] && in1==_adjInter[0])
    {
        return downTime();
    }
    else
    {
        cerr << in0->id() << " and "
             << in1->id() << " are not connected." << endl
             << "in section " << _id << ": " 
             << _adjInter[0]->id() << "->"
             << _adjInter[1]->id() << endl;
        assert(0);
        return -10;
    }
}

//======================================================================
double Section::upTime() const
{
    // 平均速度
    double av = averageUpVel();
    if (av < 1.0e-5){
        if (numUpAgents(0,length())==0)
        {
            // エージェントがいない場合は自由走行可能
            av = GVManager::getNumeric
                ("SPEED_LIMIT_SECTION")/60/60; 
        }
        else
        {
            av = 1.0e-9;
        }
    }
    double len = length();
    return len/av;
}

//======================================================================
double Section::downTime() const
{
    // 平均速度
    double av = averageDownVel();
    if (av < 1.0e-5)
    {
        if (numDownAgents(0,length())==0)
        {
            // エージェントがいない場合は自由走行可能
            av = GVManager::getNumeric
                ("SPEED_LIMIT_SECTION")/60/60; 
        }
        else
        {
            av = 1.0e-9;
        }
    }
    double len = length();
    return len/av;
}

//======================================================================
bool Section::streetTrafficWalkerTypes(vector<int> &walkerTypes)
{
    if (_streetTrafficWalkerTypes.size() == 0) 
    {
        return false;
    }
    walkerTypes = _streetTrafficWalkerTypes;
    return true;
}

//======================================================================
bool Section::mayPassWalkerType(int walkerType)
{
    if (_sidewalkWidthRight != 0
        && _sidewalkWidthLeft != 0)
    {
        return true;
    }
    return streetEntity()->mayPassWalkerType(walkerType);
}
