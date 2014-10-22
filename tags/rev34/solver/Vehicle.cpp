#include "Vehicle.h"
#include "RoadMap.h"
#include "LaneBundle.h"
#include "Intersection.h"
#include "ODNode.h"
#include "Section.h"
#include "Lane.h"
#include "RoadEntity.h"
#include "TimeManager.h"
#include "Random.h"
#include "Route.h"
#include "ARouter.h"
#include "Router.h"
#include "LocalLaneRoute.h"
#include "LocalLaneRouter.h"
#include "Signal.h"
#include "SignalColor.h"
#include "VirtualLeader.h"
#include "GVManager.h"
#include "AmuPoint.h"
#include <cassert>
#include <cmath>
#include <algorithm>

using namespace std;

//======================================================================
Vehicle::Vehicle():_id()
{
    GVManager& gvm = GVManager::instance();

    _bodyLength = 4.400;
    _bodyWidth  = 1.830;
    _bodyHeight = 1.315;
    _bodyColorR = 1.0;
    _bodyColorG = 0.0;
    _bodyColorB = 0.0;

    _roadMap      = NULL;
    _intersection = NULL;
    _prevIntersection = NULL;
    _section      = NULL;
    _lane         = NULL;
    _nextLane     = NULL;

    _length      = 0;
    _oldLength   = -10;
    _totalLength = 0;
    _tripLength  = 0;
    _error       = 0;
    _velocity      = 0;
    _errorVelocity = 0;
    _accel         = 0;
    _vMax = 60.0/60.0/60.0;
    _startTime = 0;

    _blinker.setNone();
    _shiftLane.setVehicle(this);
    _lookupShiftLane
        = (gvm.getNumeric("LOOKUP_SHIFT_LANE") == 1);
    _strictCollisionCheck
        = (gvm.getNumeric("STRICT_COLLISION_CHECK") == 1);
    _entryTime = 0;
    _isNotifying = false;
    _hasPaused   = false;
    _sleepTime   = 0;

    _route  = new Route();
    _router = new Router();
    _localRouter.setLocalRoute(&_localRoute);
    _localRoute.clear();

#ifdef USE_ADDIN
    _addinData = NULL;
#endif

#ifdef _OPENMP
    _routerSearchStep = (int)gvm.getNumeric("ROUTER_SEARCH_STEP");
    _running = false;
#endif
}

//======================================================================
void Vehicle::setId(const string& id)
{
    _id = id;
}

//======================================================================
Vehicle::~Vehicle()
{
    if(_router) delete _router;
    if(_route) delete _route;

    if(_intersection) _intersection->eraseWatchedVehicle(this);
    if(_section) _section->eraseWatchedVehicle(this);

    for (int i=0; i<static_cast<signed int>(_leaders.size()); i++)
    {
        delete _leaders[i];
    }
    _leaders.clear();

#ifdef USE_ADDIN
    if (_addinData != NULL)
        free(_addinData);
#endif
}

//======================================================================
void Vehicle::preRecognize()
{
    if (_sleepTime>0) 
    {
        return;
    }

    /**
     * 並列処理用に前処理を分ける、非並列、並列指定がないときも呼ぶ
     */

    if (_section!=NULL && _localRouter.isSearchingSideLanes())
    {
        _localRouter.localReroute(_section, _lane, _length);
    }
    if (_section!=NULL
        && _section->lengthToNext(_lane, _length)<_bodyLength
        && _lane->frontAgent(this)==NULL
        && _velocity==0
        && _hasPaused == false)
    {
        // 単路中で、交差点との境界に近く、レーンの先頭で、速度0の場合、
        // 一旦停止フラグを立てる
        _hasPaused = true;
    }

    GVManager& gvm = GVManager::instance();
    // _leadersをリセット
    for (int i=0; i<static_cast<signed int>(_leaders.size()); i++)
    {
        delete _leaders[i];
    }
    _leaders.clear();

    RelativeDirection turning = _localRoute.turning();

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 希望走行速度を決定する
    if (_section!=NULL)
    {
        _vMax = _lane->speedLimit()/60.0/60.0; //[km/h]->[m/msec]
    }
    else
    {
        _vMax = _localRoute.last()->speedLimit()/60.0/60.0;

        double tmpV = _vMax;
        if (_intersection->signal()!=NULL
            && _intersection->signal()
            ->hasPermission(_intersection
                            ->direction(_prevIntersection))>=2)
        {
            tmpV = gvm.getNumeric("VELOCITY_CRAWL")/60.0/60.0;
            if (tmpV<_vMax)
                _vMax = tmpV;
        }
        if (turning==RD_RIGHT)
        {
            tmpV = gvm.getNumeric("VELOCITY_AT_TURNING_RIGHT")/60.0/60.0;
        }
        else if (turning==RD_LEFT)
        {
            tmpV = gvm.getNumeric("VELOCITY_AT_TURNING_LEFT")/60.0/60.0;
        }
        if (tmpV<_vMax)
        {
            _vMax = tmpV;
        }
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 車線変更中の処理(車線変更が終わったかどうか)
    if (_shiftLane.isActive())
    {
        if (_shiftLane.proceedShift())
        {
            _shiftLane.endShift();
        }
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 車線変更するかどうか、バスクラスは車線変更しない
    else if (_section!=NULL && !_localRoute.desiredEmpty())
    {
        // 隣のレーンが入っているはず
        int isLeft = _localRouter.shiftDirection();

        Lane* laneTo = NULL;
        double lengthTo;
        if (isLeft==1)
        {
            _section->getLeftSideLane(_lane, _length,
                                      &laneTo, &lengthTo);
        }
        else if (isLeft==-1)
        {
            _section->getRightSideLane(_lane, _length,
                                       &laneTo, &lengthTo);
        }

        if (laneTo!=NULL
            && _shiftLane.canShift(laneTo, lengthTo))
        {
            // 車線変更開始
            _shiftLane.beginShift(laneTo, lengthTo);
        }
    }
}

//======================================================================
void Vehicle::recognize()
{
#ifndef _OPENMP
    preRecognize();
#endif

    if (_sleepTime>0)
    {
        return;
    }

    /**
     * とりあえず，エージェントは毎ステップ必要な全ての情報を
     * 正しく認知できることを 前提とする．
     * 幾何学的条件により見えなかったり見落としなどを実装するためには
     * この関数を修正する必要があろう．
     */

    GVManager& gvm = GVManager::instance();

    Intersection* nextInter = NULL;
    Intersection* prevInter = NULL;

    // 次の交差点での進行方向と進行レーンを求める
    Lane* mainLane = _localRoute.mainLaneInIntersection();
    RelativeDirection turning = _localRoute.turning();
  
    // 次の交差点に侵入するか否か
    bool isNextInterEnterable = true;

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 先行エージェントの情報、厳密位置検索
    RoadOccupant* front
        = _lane->frontAgentStrict(this->length()+_bodyLength/2, this);
    if (front!=NULL)
    {
#ifndef VL_DEBUG
        VirtualLeader* leader = 
            new VirtualLeader(front->length()-front->bodyLength()/2
                              -_length-_bodyLength/2,
                              front->velocity());
#else
        VirtualLeader* leader = 
            new VirtualLeader(front->length()-front->bodyLength()/2
                              -_length-_bodyLength/2,
                              front->velocity(),
                              "FRONT:"+front->id());
#endif
        _leaders.push_back(leader);

        // 先行車が存在し，隣のレーンが空いていて，
        // かつ隣のレーンからでもグローバル経路を満たせる場合には車線変更
        if (_section
            && (_accel<0 || _velocity==0)
            && _localRoute.desiredEmpty() )
        {
            _localRouter.localRerouteSide(_section, _lane, _length);
        }
    }
    // 先のレーンを探索する
    else
    {
        // 自分と先行エージェントの距離を求めるための変数
        // 探索したレーンの長さを次々に足していく
        double totalLength;
        LaneBundle* bundle = NULL;
        LaneBundle* bundleBegin = NULL;
        if (_section!=NULL)           bundleBegin = _section;
        else if (_intersection!=NULL) bundleBegin = _intersection;
        Lane* lookupLane;

        // 車線変更中で先行エージェントがなければ希望経路レーン検索
        // 車線変更先のレーンの次から3レーン先まで
        // 本当は全部車線変更先のレーン探索にした方がいいかもしれない
        // とりあえずレーン上の車両探索のみ追加、
        // 車線変更できずに戻る事もある
        Lane* desireBegin= _localRoute.desiredBegin();
        bundle = bundleBegin;

        if (_lookupShiftLane && _shiftLane.isActive() &&
            _shiftLane.agentToNext() == NULL && desireBegin != NULL)
        {
            totalLength = 0;
            lookupLane = _shiftLane.laneTo();

            for (int i=0; i<3; i++)
            {
                if (bundle->isNextLaneMine(lookupLane)==false)
                {
                    bundle = bundle->nextBundle(lookupLane);
                    // ODノードに到達したら探索を打ち切る
                    if (dynamic_cast<ODNode*>(bundle)!=NULL)
                    {
                        break;
                    }
                }
                totalLength += lookupLane->length();

                // 次のレーンを探す
                // 希望経路レーンが車線変更レーンの向こうなら
                // 直進レーンを検索
                Lane* nextLane = NULL;
                if (desireBegin == _shiftLane.laneTo())
                {
                    nextLane = _localRoute.desiredNext(lookupLane);
                }
                if (nextLane == NULL)
                {
                    nextLane = bundle->mostStraightNextLane(lookupLane);
                }
                lookupLane = nextLane;

                if (lookupLane==NULL)
                {
                    break;
                }

                // 最初のレーンのみ厳密位置検索
                if (i == 0)
                {
                    front = lookupLane->tailAgentStrict(this);
                }
                else
                {
                    front = lookupLane->tailAgent(this);
                }
                if (front != NULL)
                {
#ifndef VL_DEBUG
                    VirtualLeader* leader =
                        new VirtualLeader(totalLength+front->length()
                                          -front->bodyLength()/2
                                          - _length-_bodyLength/2,
                                          front->velocity());
#else
                    VirtualLeader* leader =
                        new VirtualLeader(totalLength+front->length()
                                          -front->bodyLength()/2
                                          - _length-_bodyLength/2,
                                          front->velocity(),
                                          "SHIFTFRONT:"+front->id());
#endif
                    _leaders.push_back(leader);
                }
            }
        }

        // 3レーン先まで探索
        totalLength = 0;
        lookupLane = _lane;
        bundle = bundleBegin;
        for (int i=0; i<3; i++)
        {
            if (bundle->isNextLaneMine(lookupLane)==false)
            {
                bundle = bundle->nextBundle(lookupLane);
                // ODノードに到達したら探索を打ち切る
                if (dynamic_cast<ODNode*>(bundle)!=NULL)
                {
                    break;
                }
            }
            totalLength += lookupLane->length();

            // 次のレーンを探す
            Lane* nextLane = _localRoute.next(lookupLane);
            if (nextLane==NULL)
            {
                nextLane = bundle->mostStraightNextLane(lookupLane);
            }
            lookupLane = nextLane;

            if (lookupLane==NULL)
            {
                break;
            }

            // 対象のレーンの制限速度が現在の速度よりも小さければ
            // 事前に減速する
            if (lookupLane->speedLimit()/60.0/60.0<_velocity)
            {
#ifndef VL_DEBUG
                VirtualLeader* leader =
                    new VirtualLeader(totalLength-_length-_bodyLength/2,
                                      lookupLane->speedLimit()/60.0/60.0);
#else
                VirtualLeader* leader =
                    new VirtualLeader(totalLength-_length-_bodyLength/2,
                                      lookupLane->speedLimit()/60.0/60.0,
                                      "SPEEDLIMIT:"+lookupLane->id());
#endif
                _leaders.push_back(leader);
            }

            // 最初のレーンのみ厳密位置検索
            if (i == 0)
            {
                front = lookupLane->tailAgentStrict(this);
            }
            else
            {
                front = lookupLane->tailAgent(this);
            }
            if (front != NULL)
            {
#ifndef VL_DEBUG
                VirtualLeader* leader =
                    new VirtualLeader(totalLength+front->length()
                                      -front->bodyLength()/2
                                      - _length-_bodyLength/2,
                                      front->velocity());
#else
                VirtualLeader* leader =
                    new VirtualLeader(totalLength+front->length()
                                      -front->bodyLength()/2
                                      - _length-_bodyLength/2,
                                      front->velocity(),
                                      "FRONT:"+front->id());
#endif
                _leaders.push_back(leader);
            }

            // lookupLaneが合流後のレーン(numPreviousLanes>1)の場合は
            // 合流するレーンも見てみる
            if (bundle->numPreviousLane(lookupLane)>1)
            {
                vector<Lane*> prevLanes
                    = bundle->previousLanes(lookupLane);
                for (int i=0;
                     i<static_cast<signed int>(prevLanes.size()); i++)
                {
                    if (prevLanes[i]==_lane) continue;
                    Vehicle* merge = prevLanes[i]->headVehicle();
                    if (merge!=NULL)
                    {
                        if ((prevLanes[i]->length()-merge->length())
                            *_velocity
                            <(totalLength-_length)*
                            merge->velocity())
                        {
                            front = merge;
#ifndef VL_DEBUG
                            VirtualLeader* leader =
                                new VirtualLeader
                                (totalLength+front->length()
                                 - front->bodyLength()/2
                                 + lookupLane->length()
                                 - prevLanes[i]->length()
                                 - _length-_bodyLength/2,
                                 front->velocity());
#else
                            VirtualLeader* leader =
                                new VirtualLeader
                                (totalLength+front->length()
                                 - front->bodyLength()/2
                                 + lookupLane->length()
                                 - prevLanes[i]->length()
                                 - _length-_bodyLength/2,
                                 front->velocity(),
                                 "MERGE:"+front->id());
#endif
                            _leaders.push_back(leader);
                        }
                    }
                }
            }
            if (front!=NULL)
            {
                break;
            }
        }
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 車線変更をされる場合
    /*
     * 車線変更中の車両で自分の直前にいるものを探索し
     * VirtualLeaderに加える．
     */
    if (_section!=NULL)
    {
        _shiftLane.searchInterruption();
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 信号によって停止すべき位置を取得
    // 単路の先頭車両だけで十分?
    Signal* signal = NULL;
    // 「次の」信号で停止するか否か
    if (_section!=NULL)
    {
        // 単路が接続する交差点
        nextInter = _section->intersection(_section->isUp(_lane));
        prevInter = _section->intersection(!_section->isUp(_lane));
    
        if (dynamic_cast<ODNode*>(nextInter)==NULL
            && mainLane == NULL)
        {
            print();
        }
    }
    else
    {
        // 交差点にいる場合...次に通過する交差点
        // 希望したレーンを守れていない場合は実際に走行する経路に従う
        if (!_localRoute.desiredEmpty())
        {
            // 通常、LocalRouteの最後から2つめには
            // 交差点内の最後のレーンが入っているはず
            Lane* exitLane = _localRoute.previous(_localRoute.last());
            if (exitLane && _intersection->isMyLane(exitLane))
            {
                nextInter = _intersection
                    ->next(_intersection
                           ->direction(exitLane->endConnector()));
            }
            else
            {
                nextInter = _intersection->next(1);
            }
        }
        else if(_prevIntersection == NULL)
        {
            nextInter = _route->next(_intersection);
        }
        else
        {
            nextInter = _route->next(_prevIntersection, _intersection);
        }
        prevInter = _intersection;
    }

    if (nextInter!=NULL)
    {
        // 次の信号の情報を取得
        signal = nextInter->signal();
        if (signal != NULL)
        {
            if (_section!=NULL)
            {
                switch (nextInter->hasPermission(mainLane, this))
                {
                case 0:
                { // 進入許可なし（赤信号）
#ifndef VL_DEBUG
                    VirtualLeader* leader =
                        new VirtualLeader(_section
                                          ->lengthToNext(_lane, _length)
                                          -_bodyLength/2,
                                          0);
#else
                    VirtualLeader* leader =
                        new VirtualLeader(_section
                                          ->lengthToNext(_lane, _length)
                                          -_bodyLength/2,
                                          0,
                                          "RED:"+nextInter->id());
#endif
                    _leaders.push_back(leader);
                    isNextInterEnterable = false;
                    break;
                }
                case 1:   // 進入許可あり（青信号）
                    break;
                case 2:
                { // 徐行制限
#ifndef VL_DEBUG
                    VirtualLeader* leader =
                        new VirtualLeader
                        (_section->lengthToNext(_lane, _length)
                         +_bodyLength,
                         gvm.getNumeric("VELOCITY_CRAWL")/60/60);
#else
                    VirtualLeader* leader =
                        new VirtualLeader
                        (_section->lengthToNext(_lane, _length)+_bodyLength,
                         gvm.getNumeric("VELOCITY_CRAWL")/60/60,
                         "YELLOBLINK:"+nextInter->id());
#endif
                    _leaders.push_back(leader);
                    break;
                }
                case 3:
                { // 一旦停止
                    if (_hasPaused)
                    {
#ifndef VL_DEBUG
                        VirtualLeader* leader =
                            new VirtualLeader
                            (_section->lengthToNext(_lane, _length)
                             +_bodyLength,
                             gvm.getNumeric("VELOCITY_CRAWL")/60/60);
#else
                        VirtualLeader* leader =
                            new VirtualLeader
                            (_section->lengthToNext(_lane, _length)
                             +_bodyLength,
                             gvm.getNumeric("VELOCITY_CRAWL")/60/60,
                             "REDBLINK1:"+nextInter->id());
#endif
                        _leaders.push_back(leader);
                    }
                    else
                    {
#ifndef VL_DEBUG
                        VirtualLeader* leader =
                            new VirtualLeader
                            (_section->lengthToNext(_lane, _length)
                             -_bodyLength/2, 0);
#else
                        VirtualLeader* leader =
                            new VirtualLeader
                            (_section->lengthToNext(_lane, _length)
                             -_bodyLength/2, 0,
                             "REDBLINK:"+nextInter->id());
#endif
                        _leaders.push_back(leader);
                        isNextInterEnterable = false;
                        break;
                    }
                    break;
                }
                default:
                    break;
                }
            }
            else if (_intersection!=NULL &&
                     (signal->mainColor(nextInter->direction(prevInter))
                      == SignalColor::red()
                      || signal->mainColor(nextInter->direction(prevInter))
                      == SignalColor::redblink()))
            {
#ifndef VL_DEBUG
                VirtualLeader* leader =
                    new VirtualLeader(_intersection
                                      ->nextSection(nextInter)->length()
                                      -_bodyLength/2,
                                      0);
#else
                VirtualLeader* leader =
                    new VirtualLeader(_intersection
                                      ->nextSection(nextInter)->length()
                                      -_bodyLength/2,
                                      0,
                                      "RED:"+nextInter->id());
#endif
                _leaders.push_back(leader);
                isNextInterEnterable = false;
            }
        }
    }

    // もし次の交差点を直進する場合で，信号が青のときは
    // もう一つ先の交差点を見る
    // - さらに先を見る必要はある？
    if (isNextInterEnterable==true
        && _section!=NULL
        && _route->next(prevInter, nextInter)!=NULL
        && turning == RD_STRAIGHT)
    {
        Lane* exitLane = _localRoute.previous(_localRoute.last());
        if (exitLane && nextInter->isMyLane(exitLane))
        {
            Intersection* tmpInter = nextInter
                ->next(nextInter->direction(exitLane->endConnector()));
            prevInter = nextInter;
            nextInter = tmpInter;
            signal = nextInter->signal();

            if (signal!=NULL
                && (signal->mainColor(nextInter->direction(prevInter))
                    == SignalColor::red()
                    || signal->mainColor(nextInter->direction(prevInter))
                    == SignalColor::redblink()))
            {
                // mainLaneを使った判断はできない
                // 本来は交差点内のレーンも考慮すべき
#ifndef VL_DEBUG
                VirtualLeader* leader =
                    new VirtualLeader(_section->lengthToNext(_lane, _length)
                                      +nextInter->nextSection(prevInter)
                                      ->length()
                                      -_bodyLength/2,
                                      0);
#else
                VirtualLeader* leader =
                    new VirtualLeader(_section->lengthToNext(_lane, _length)
                                      +nextInter->nextSection(prevInter)
                                      ->length()
                                      -_bodyLength/2, 0,
                                      "RED:"+nextInter->id());
#endif
                _leaders.push_back(leader);
            }
        }
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 交差点右左折時には最小ヘッドウェイ（時間）が定義されており，
    // 車両はこれから進入しようとするmainLaneに最後に車両が進入してから
    // 少なくとも最小ヘッドウェイ分だけ間隔を空けなければならない
    if (isNextInterEnterable
        && _section!=NULL
        && _section->isHeadAgent(this, _lane)
        && (turning==RD_LEFT || turning==RD_RIGHT)
        && _velocity!=0
        && TimeManager::time()
        +_section->lengthToNext(_lane, _length)/_velocity
        -_nextLane->lastArrivalTime()
        < gvm.getNumeric("MIN_HEADWAY_AT_TURNING")*1000) { //[sec]->[msec]
#ifndef VL_DEBUG
        VirtualLeader* leader =
            new VirtualLeader(_section->lengthToNext(_lane, _length)
                              -_bodyLength/2, 0);
#else
        VirtualLeader* leader =
            new VirtualLeader(_section->lengthToNext(_lane, _length)
                              -_bodyLength/2, 0,
                              "HEADWAY:"+nextInter->id());
#endif

        _leaders.push_back(leader);
        isNextInterEnterable = false;
    }
 
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 交錯する車両の認知
    vector<Lane*> clInter;   // 交差点内交錯レーン
    vector<Lane*> clSection; // 上流の単路内レーン
 
    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 交差点内交錯レーン
    // 単路走行中の先頭車両のみ判定する
    if (isNextInterEnterable==true
        && _section!=NULL
        && _section->isHeadAgent(this, _lane))
    {
        nextInter = _section->intersection(_section->isUp(_lane));

        // ODノードである場合は判定しない
        if (nextInter!=NULL && dynamic_cast<ODNode*>(nextInter)==NULL)
        {

            // 交差点内の交錯レーン（分岐、合流含む）clInterと
            // 交錯レーンの上流の単路内レーンclSectionを同時に取得する
            // この時点ではレーンの幾何的情報のみを元に抽出
            nextInter->collisionLanes(_localRoute.lanesInIntersection(),
                                      &clInter, &clSection);
 
            for (unsigned int i=0; i<clInter.size(); i++)
            {

                // エージェントでなく車両検索
                // 交錯を厳密に評価する際はすべての車両をチェック     
                // そうでなければtailで判断
                Vehicle* colVehicleBegin = clInter[i]->tailVehicle();
                for (Vehicle* colVehicle = colVehicleBegin;
                     colVehicle != NULL; 
                     colVehicle = clInter[i]->frontVehicle(colVehicle))
                {
                    if (!_strictCollisionCheck
                        && colVehicle != colVehicleBegin)
                    {
                        break;
                    }

                    // 認知エラーが指定されている場合は無視する
                    if (colVehicle!=NULL && colVehicle!=this)
                    {
                        // 交錯する地点までの距離を計算
                        // 相手車両が交錯地点を既に通過している場合は
                        // 無視して構わない
                        bool hasPassedCollisionPoint = false;

                        const vector<Lane*>* liInter
                            = _localRoute.lanesInIntersection();
                        for (unsigned int j=0; j<liInter->size(); j++)
                        {
                            AmuPoint crossPoint;
                            if ((*liInter)[j]
                                ->createIntersectionPoint
                                (clInter[i]->lineSegment(),
                                 &crossPoint))
                            {
                                double lengthToCollide
                                    = clInter[i]->lineSegment()
                                    .pointBegin().distance(crossPoint);
                                if (lengthToCollide<colVehicle
                                    ->length()-colVehicle->bodyLength()/2)
                                {
                                    hasPassedCollisionPoint = true;
                                }
                            }
                            // 交点を持たない場合は
                            // 相手車両が交錯レーンの上流にいるため
                            // hasPassedCollisionPointはfalseのまま
                        }
	    
                        if (!hasPassedCollisionPoint)
                        {
                            double tti[2];
                            // 自分が交差点に到達するまでの時間
                            if (_section->lengthToNext(_lane, _length)<30)
                            {
                                tti[0] = 0.0;
                            }
                            else if (_velocity!=0)
                            {
                                tti[0]
                                    = _section->lengthToNext(_lane, _length)
                                    / _velocity;
                            }
                            else
                            {
                                tti[0] = 100.0*1000;
                            }
                            // 相手が交差点を通過し終えるまでの時間
                            if (colVehicle->velocity()==0)
                            {
                                tti[1] = 100.0*1000;
                            }
                            else 
                            {
                                tti[1]
                                    = nextInter->lengthToNext
                                    (clInter[i],
                                     colVehicle->length())
                                    / colVehicle->velocity();
                            }
                            if (tti[0]<tti[1])
                            {
#ifndef VL_DEBUG
                                VirtualLeader* leader =
                                    new VirtualLeader
                                    (_section->lengthToNext(_lane, _length)
                                     -_bodyLength/2, 0);
#else
                                VirtualLeader* leader =
                                    new VirtualLeader
                                    (_section->lengthToNext(_lane, _length)
                                     -_bodyLength/2, 0,
                                     "CLINT:"+colVehicle->id());
#endif
                                _leaders.push_back(leader);
                                isNextInterEnterable = false;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 交錯単路（右直関係など）
    // 単路の先頭車両だけで十分
    /*
     * 交錯単路を走行する車両との判定は以下の手順で行っている
     * 0. 交差点内交錯レーンの上流にある単路内レーンが上は取得済みである
     *
     * 1. 取得した単路内レーンについて、それぞれの先頭車両を取得する
     *
     * 2. 「先頭車両を（目視などによって）認識できる場合に限り」、
     *    自車と交錯単路内レーンの先頭車両との位置関係、
     *    進行方向（ウィンカーで判別）から
     *    道を譲るべきかどうか判定(_isYielding)
     *
     * 3. 道を譲ると判定され、実際に該当交差点に到達するまでの時間差が
     *    小さい場合に減速->停止
     *    （相手よりもかなり前に自分が交差点を通過できる場合は
     *      優先関係を無視する）
     *
     * 自車と同方向（同じ単路）から進入する車には注意が必要。
     * _isYieldingの中で条件分岐して対応している。
     * （同方向であっても交錯する可能性はある。
     *   道路中央を走行する路面電車が左折するような状況。）
     */
    if (isNextInterEnterable==true
        && _section!=NULL
        && !clSection.empty()
        && _section->isHeadAgent(this, _lane))
    {
        nextInter = _section->intersection(_section->isUp(_lane));
        if (nextInter!=NULL)
        {
            int thisDir = nextInter->direction(_section);
            signal = nextInter->signal();

            for (unsigned int i=0; i<clSection.size(); i++)
            {
                if (!clSection[i])
                {
                    assert(0);
                }
                int thatDir
                    = nextInter->direction(clSection[i]->endConnector());
                assert(thatDir!=-1);

                // 対象とする単路の信号が青（または黄，黄点滅）
                /*
                 * 赤点滅は無視する（無条件で自分が優先される）
                 */
                if (signal->hasPermission(thatDir)==1
                    || signal->hasPermission(thatDir)==2 )
                {
                    // レーンの先頭車両、エージェントでなく車両検索
                    Vehicle* head = clSection[i]->headVehicle();

                    // 相手車両が走行中の単路
                    Section* collisionSection
                        = nextInter->nextSection(thatDir);

                    if (head!=NULL
                        && head!=this
                        /*&& _isVisible(head)*/)
                    {
                        // この時点で見通しを考慮するならコメントを外す
                        double gap
                            = gvm.getNumeric
                            ("GAP_ACCEPTANCE_VEHICLE_CROSS")*1000;
                        // [sec]->[msec]
                        if (_isYielding(nextInter, thisDir, thatDir, turning, head))
                        {
                            double tti[2];
                            // 自車のtime to intersection
                            if (_section->lengthToNext(_lane, _length)
                                <_bodyLength)
                            {
                                // 交差点に近ければ3秒（適当）
                                tti[0] = 3.0*1000;
                            }
                            else if (_velocity!=0)
                            {
                                // 交差点から遠ければ実時間
                                tti[0]
                                    = _section->lengthToNext(_lane, _length)
                                    / _velocity;
                            }
                            else
                            {
                                // ただし_velocity=0だと
                                // 無限大になってしまうため100秒に（適当）
                                tti[0] = 100.0*1000;
                            }

                            // 相手車両のtime to intersection
                            // 相手が交差点の直前で加速中は別処理
                            // （優先車両の動き出し）
                            /*
                             * 動き出したばかりで低速の状態であると
                             * tti[1]を過大に評価してしまうため
                             */
                            if (collisionSection->lengthToNext
                                (clSection[i], head->length())
                                < head->bodyLength()
                                && head->velocity()>0)
                            {
                                tti[1] = 0.0;
                            }
                            else if (head->velocity()!=0)
                            {
                                tti[1]
                                    = collisionSection
                                    ->lengthToNext(clSection[i],
                                                   head->length())
                                    / head->velocity();
                            }
                            else
                            {
                                tti[1] = 50.0*1000;
                            }

                            if (tti[1]-tti[0] < gap)
                            {
                                /*
                                 * 相手に道を譲るべき状況で，かつ
                                 * 自分が交差点に到達するまでの時間
                                 * (TTI:time to intersection)
                                 * + gap(GAP_ACCEPTANCE)よりも
                                 * 相手が先に交差点に到達する場合に停車
                                 */
#ifndef VL_DEBUG
                                VirtualLeader* leader =
                                    new VirtualLeader
                                    (_section->lengthToNext(_lane, _length)
                                     -_bodyLength/2, 0);
#else
                                VirtualLeader* leader =
                                    new VirtualLeader
                                    (_section->lengthToNext(_lane, _length)
                                     -_bodyLength/2, 0,
                                     "CLSEC:"+head->id());
#endif
                                _leaders.push_back(leader);
                                isNextInterEnterable = false;
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 交差点を右左折する場合にはあらかじめ減速する
    if (isNextInterEnterable==true
        && _section!=NULL
        && turning!=RD_STRAIGHT)
    {
        if (turning==RD_RIGHT
            && _velocity>gvm.getNumeric("VELOCITY_AT_TURNING_RIGHT")/60/60)
        {
#ifndef VL_DEBUG
            VirtualLeader* leader =
                new VirtualLeader
                (_section->lengthToNext(_lane, _length)+_bodyLength*2,
                 gvm.getNumeric("VELOCITY_AT_TURNING_RIGHT")/60/60);
#else
            VirtualLeader* leader =
                new VirtualLeader
                (_section->lengthToNext(_lane, _length)+_bodyLength*2,
                 gvm.getNumeric("VELOCITY_AT_TURNING_RIGHT")/60/60,
                 "RTURN:"+_section->id());
#endif
            _leaders.push_back(leader);
        }
        else if (turning==RD_LEFT
                 && _velocity>gvm.getNumeric
                 ("VELOCITY_AT_TURNING_LEFT")/60/60)
        {
#ifndef VL_DEBUG
            VirtualLeader* leader =
                new VirtualLeader
                (_section->lengthToNext(_lane, _length)+_bodyLength*2,
                 gvm.getNumeric("VELOCITY_AT_TURNING_LEFT")/60/60);
#else
            VirtualLeader* leader =
                new VirtualLeader
                (_section->lengthToNext(_lane, _length)+_bodyLength*2,
                 gvm.getNumeric("VELOCITY_AT_TURNING_LEFT")/60/60,
                 "LTURN:"+_section->id());
#endif
            _leaders.push_back(leader);
        }
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 自身がレーンの先頭で，かつ現在単路内を走行し，
    // 先の単路内のレーン最後尾の車が交差点に近い位置に停車しているとき
    // 渋滞が交差点内部まで延伸するのを防ぐために交差点手前で停止する
    if (isNextInterEnterable==true
        && _lane->frontAgent(this->length()+_bodyLength/2, this)==NULL
        && _section!=NULL
        && dynamic_cast<ODNode*>
        (_section->intersection(_section->isUp(_lane)))==NULL)
    {
        // 前方レーンに必要なスペースを計算する
        // 自分自身が停車するのに必要なスペース(+2は車間距離)
        double enoughLength = _bodyLength+2;

        // _laneから_localRoute.last()までの間（交差点内）を
        // 走行中の車両の車長+車間距離を加える
        Lane* targetLane = _localRoute.next(_lane);
        while (targetLane!=_localRoute.last())
        {
            RoadOccupant* agent = targetLane->tailAgent(this);
            while (agent)
            {
                enoughLength += agent->bodyLength()+2;
                // もう一つ先のエージェントを取得
                agent = targetLane->frontAgent(agent, this);
            }
            // もう一つ先のレーンを取得
            targetLane = _localRoute.next(targetLane);
        }

        // _localRouteの最後のレーンは次の単路の最初のレーンのはず
        RoadOccupant* tail = (_localRoute.last())->tailAgent(this);
        if (tail!=NULL
            && tail->length()-tail->bodyLength()/2 < enoughLength
            && tail->velocity()
            < gvm.getNumeric("VELOCITY_AT_TURNING_RIGHT")*0.5/60/60){

#ifndef VL_DEBUG
            VirtualLeader* leader =
                new VirtualLeader(_section->lengthToNext(_lane, _length)
                                  -_bodyLength/2, 0);
#else
            VirtualLeader* leader =
                new VirtualLeader(_section->lengthToNext(_lane, _length)
                                  -_bodyLength/2, 0,
                                  "TAIL:"+tail->id());
#endif
            _leaders.push_back(leader);
            isNextInterEnterable = false;
            if (_velocity==0)
            {
                _sleepTime = 1500;
            }
        }
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 交差点内での速度制御
    // 交差点内の交錯レーンを車両が走行している場合
    // 流入した境界番号の優先関係によって道を譲る
    /* これによって合流に対応する（つもり） */
    if (_intersection!=NULL)
    {
        int thisDir = _intersection->direction(_prevIntersection);    

        // レーンの幾何的情報のみを元に抽出
        // 現在のレーン＋前のレーンをチェック
        const std::vector<Lane*>* lanesAll
            = _localRoute.lanesInIntersection();
        std::vector<Lane*> lanesFront;
        unsigned int curLanePos;
        for (curLanePos = 0; curLanePos < lanesAll->size(); curLanePos++)
        {
            if (lanesAll->at(curLanePos) == _lane)
            { 
                break;
            }
        }
        for (; curLanePos < lanesAll->size(); curLanePos++)
        {
            lanesFront.push_back(lanesAll->at(curLanePos));
        }
        _intersection->collisionLanes(&lanesFront, &clInter, &clSection);

        for (unsigned int i=0; i< clInter.size(); i++)
        {
            // エージェントでなく車両検索
            // 交錯を厳密に評価する際は重なる可能性があるので
            // すべての車両をチェック     
            // そうでなければ道を譲ることを考えるためtailで判断、
            Vehicle* colVehicleBegin = clInter[i]->tailVehicle();
            for (Vehicle* colVehicle = colVehicleBegin;
                 colVehicle != NULL; 
                 colVehicle = clInter[i]->frontVehicle(colVehicle))
            {
                if (!_strictCollisionCheck
                    && colVehicle != colVehicleBegin)
                {
                    break;
                }
                int thatDir = colVehicle->directionFrom();

                // 認知エラーが指定されていた場合は認知しない
                if (_isYielding(_intersection, thisDir, thatDir,
                                turning, colVehicle))
                {
                    // レーン同士の交点の位置を計算
                    Lane* targetLane = NULL;
                    const vector<Lane*>* thatLanes
                        = colVehicle->lanesInIntersection();
                    vector<Lane*>::const_iterator itl
                        = find(thatLanes->begin(),
                               thatLanes->end(),
                               clInter[i]);
                    assert(itl!=thatLanes->end());
	  
                    AmuPoint crossPoint;
                    Lane* thisCL = NULL;   // 交錯点を持つ自分側のレーン
                    Lane* thatCL = NULL;   // 交錯点を持つ相手側のレーン 
                    // thisCLにおける始点から交錯点までの距離
                    double thisLength = 0;
                    // thatCLにおける始点から交錯点までの距離
                    double thatLength = 0;

                    for (;itl!=thatLanes->end(); itl++)
                    {
                        for (targetLane=_lane;
                             _intersection->isMyLane(targetLane);
                             targetLane = _localRoute.next(targetLane))
                        {
                            if (targetLane
                                ->createIntersectionPoint
                                ((*itl)->lineSegment(), &crossPoint))
                            {
                                thisCL = targetLane;
                                thatCL = (*itl);
                                thisLength =
                                    thisCL->lineSegment()
                                    .pointBegin().distance(crossPoint);
                                thatLength =
                                    thatCL->lineSegment()
                                    .pointBegin().distance(crossPoint);
                            }
                            if (thisCL)
                            {
                                break;
                            }
                        }
                        if (thatCL)
                        {
                            break;
                        }
                    }

                    if (thisCL == NULL || thatCL == NULL)
                    {
                        // 交錯なし
                    }  
                    else if ((thisCL==_lane
                              && thisLength<_length-_bodyLength/2)
                             || (thatCL==colVehicle->lane()
                                 && thatLength
                                 < colVehicle->length()
                                 - colVehicle->bodyLength()/2))
                    {
                        // 自分と相手のどちらかが
                        // 既に交錯点を通過した後であれば無視する
                    }
                    else
                    {
                        // thisCLから遡って交錯点までの距離を算出する
                        double lengthToStop = thisLength;
                        while (targetLane!=_lane)
                        {
                            targetLane = _localRoute.previous(targetLane);
                            assert(_intersection->isMyLane(targetLane));
                            lengthToStop += targetLane->length();
                        }
                        lengthToStop -= _length;

#ifndef VL_DEBUG
                        VirtualLeader* leader =
                            new VirtualLeader(lengthToStop
                                              -_bodyLength/2, 0);
#else
                        VirtualLeader* leader =
                            new VirtualLeader(lengthToStop
                                              -_bodyLength/2, 0,
                                              "CLINT2:"+colVehicle->id());
#endif
                        _leaders.push_back(leader);
                    }
                }
            }
        }
    }
}

//======================================================================
void Vehicle::readyToRun()
{

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 車線変更時は車線に対して垂直方向(error方向)の速度を生じさせる
    // これはスリープ判定より前でなければならない．
    if (_shiftLane.isActive())
    {
        _errorVelocity = _shiftLane.activeErrorVelocity();
    }
    else
    {
        _errorVelocity = 0.0;
    }

    if (_sleepTime>0)
    {
        return;
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 加速度決定

    // 加速度候補を入れるvector
    vector<double> accelCandidates;

    double vOpt;  //単位は[m/msec]
    double aOpt;  //d^2x/dt^2
    double accel;

    /**
     * 詳しくはGeneralized Force Modelを参照
     * D.Helbing and B.Tilch
     * "Generalized force model of traffic dynamics"
     * Phy.Rev.E Vol.58 No.1(1998), pp.133-138
     */
    GVManager& gvm = GVManager::instance();
    double ta0 = 2450;     //[msec]
    double ta1 = 770;      //[msec]
    double ta2;
    double ra0 =  5.59;    //[m]
    double ra1 = 98.78;    //[m]
    double da  =  1.38;    //[m]      <- 最小車間距離
    double tt  =
        gvm.getNumeric("REACTION_TIME_VEHICLE")*1000.0;
    //[sec]->[msec]   <- 反応遅れ時間

    // 自由走行（前方の状況を考慮しない）
    aOpt = (_vMax-_velocity)/ta0;
    accelCandidates.push_back(aOpt);

    // 前方の状況に応じた最適な加速度の決定
    for (int i=0; i<static_cast<signed int>(_leaders.size()); i++)
    {
        ta2 = ta1
            * exp((_leaders[i]->distance()-(da+tt*_velocity))/ra1);
        if (_velocity>_leaders[i]->velocity())
        {
            vOpt = ( ta2
                     * _vMax * (1-exp(-(_leaders[i]->distance()
                                        -(da+tt*_velocity))/ra0))
                     + ta0 * _leaders[i]->velocity() ) / (ta0+ta2);
            aOpt = (vOpt-_velocity) * (1/ta0+1/ta2);
        }
        else {
            vOpt = _vMax*(1-exp(-(_leaders[i]->distance()
                                  -(da+tt*_velocity))/ra0));
            aOpt = (vOpt-_velocity) * (1/ta0);
        }
        accelCandidates.push_back(aOpt);
    }

    // 最も遅い候補を選ぶ
    // 効用を考えたければ速度と効用のpairを使う？
    assert(!accelCandidates.empty());
    accel = *min_element(accelCandidates.begin(),
                         accelCandidates.end());
    
    // 最大加速度，最大減速度を越えた場合の処理
    if (accel>_maxAcceleration)
    {
        accel = _maxAcceleration;
    }
    else if (accel<_maxDeceleration)
    {
        accel = _maxDeceleration;
    }
    _accel = accel;

    _velocity = _velocity+accel*TimeManager::unit();

    if (_velocity < 1.0e-5)
    {
        _velocity = 0.0;
    }
}

//======================================================================
void Vehicle::run()
{
#ifdef _OPENMP
    // 移動処理中設定
    // 経過時間変更交差点と注目（車線変更）車両レーン変更クリア
    _running = true;
    _passTimeIntersection = NULL;
    _changeWatchedLane = CWLT_NONE;
    _changeWatchedLaneIntersection = NULL;
    _changeWatchedLaneSection = NULL;
#endif

    if (_sleepTime>0)
    {
        _sleepTime-=TimeManager::unit();
    }

    double length = _velocity * TimeManager::unit();
    _oldLength = _length;
    _length      += length;
    _totalLength += length;
    _tripLength  += length;
    _error += _errorVelocity * TimeManager::unit();

    if (_length < _lane->length())
    {
        // 同一レーンに登録する
        _lane->putAgent(this);

        // 単路にいる場合
        // 次の交差点まで50m以内で，次に右左折するときはウィンカーを点ける
        if (_section!=NULL /*&& _localRouter.isSearched()*/)
        {
            if (_section->lengthToNext(_lane, _length)<=50
                && _blinker.turning()!=_localRoute.turning())
            {
                // 交差点拡張方向を導入した場合は要修正
                if (_localRoute.turning()==RD_LEFT)
                {
                    _blinker.setLeft();
                }
                else if (_localRoute.turning()==RD_RIGHT)
                {
                    _blinker.setRight();
                }
                else
                {
                    _blinker.setNone();
                }
            }
        }
    }
    else {
        // 次のレーンに移る
        // 次に所属するレーンを基準にするので，oldLengthはマイナス
        _oldLength -= _lane->length();
        _length -= _lane->length();

        // _lane, _intersection, _sectionの更新
        if (_section==NULL)
        {
            assert(_intersection!=NULL);

            // 一旦停止フラグの解除
            _hasPaused = false;

            if (_intersection->isMyLane(_nextLane))
            {
                // 交差点から交差点へ
                // （想定していない）
                _runIntersection2Intersection();
            }
            else
            {
                // 交差点から単路へ
                _totalLength = 0;
                _runIntersection2Section();

                // ウィンカーを消す
                if (_blinker.isLeft() || _blinker.isRight())
                {
                    _blinker.setNone();
                }
            }
        }
        else
        {
            assert(_section!=NULL);
            if (_section->isMyLane(_nextLane))
            {
                // 単路から単路へ
                // （想定していない）
                _runSection2Section();

            }
            else
            {
                // 単路から交差点へ
                _totalLength = 0;
                _runSection2Intersection();
            }
        }
        _lane->putAgent(this);
        _lane->setLastArrivalTime(TimeManager::time());
    }
}

#ifdef _OPENMP
//======================================================================
void Vehicle::postRun()
{
    /// 移動処理中解除
    _running = false;

    /// 経過時間変更、非並列
    if (_passTimeIntersection != NULL)
    {
        _passTimeIntersection->addPassTime(_passTimeFrom,
                                           _passTimeTo,
                                           _passTime);
    }

    // 注目（車線変更）車両レーン変更、非並列
    if (_changeWatchedLane == CWLT_ERASE_SECTION)
    {
        _changeWatchedLaneSection->eraseWatchedVehicle(this);
    }
    else if (_changeWatchedLane == CWLT_ERASE_INTERSECTION)
    {
        _changeWatchedLaneIntersection->eraseWatchedVehicle(this);
    }
    else if (_changeWatchedLane == CWLT_INTERSECTION_TO_SECTION)
    {
        _changeWatchedLaneIntersection->eraseWatchedVehicle(this);
        _changeWatchedLaneSection->addWatchedVehicle(this);
    }
    else if (_changeWatchedLane == CWLT_SECTION_TO_INTERSECTION)
    {
        _changeWatchedLaneSection->eraseWatchedVehicle(this);
        if (dynamic_cast<ODNode*>(_changeWatchedLaneIntersection)==NULL)
        {
            // ODノードに入る(次のステップで消える)ときにはaddしない
            _changeWatchedLaneIntersection->addWatchedVehicle(this);
        }
    }
}
#endif

//======================================================================
const string&  Vehicle::id() const
{
    return _id;
}

//======================================================================
VehicleType Vehicle::type() const
{
    return _type;
}

//======================================================================
void Vehicle::setType(VehicleType type)
{
    _type = type;
}

//======================================================================
double Vehicle::bodyWidth() const
{
    return _bodyWidth;
}

//======================================================================
double Vehicle::bodyLength() const
{
    return _bodyLength;
}

//======================================================================
double Vehicle::bodyHeight() const
{
    return _bodyHeight;
}

//======================================================================
void Vehicle::setBodySize(double length, double width, double height) 
{
    _bodyLength = length;
    _bodyWidth = width;
    _bodyHeight = height;
}

//======================================================================
void Vehicle::setPerformance(double accel, double brake)
{
    assert(accel>0 && brake<0);
    _maxAcceleration = accel*1.0e-6;
    _maxDeceleration = brake*1.0e-6;
}

//======================================================================
void Vehicle::setBodyColor(double r, double g, double b)
{
    _bodyColorR = r;
    _bodyColorG = g;
    _bodyColorB = b;
}
//======================================================================
void Vehicle::getBodyColor(double* result_r,
                           double* result_g,
                           double* result_b) const
{
    *result_r = _bodyColorR;
    *result_g = _bodyColorG;
    *result_b = _bodyColorB;
}

//======================================================================
bool Vehicle::addToSection(RoadMap* roadMap,
                           Section* section,
                           Lane* lane,
                           double length)
{
    // 車が初めて登場したときにだけ呼ばれる
    bool check = false;
    if (roadMap!=NULL)
    {
        _roadMap = roadMap;
        _section = section;
        _prevIntersection = section->intersection(!(section->isUp(lane)));
        _length = length;
        _localRouter.setRoadMap(roadMap);
        _localRouter.setRoute(_route);
        _localRouter.setLocalRoute(&_localRoute);
        _entryTime = TimeManager::time();
        check = setLane(section, lane, length);
#ifndef GENERATE_VELOCITY_0
        if (lane->tailAgent())
        {
            _velocity = lane->tailAgent()->velocity();
        }
        else
        {
            _velocity = lane->speedLimit()/60/60;
        }
#endif
        GVManager& gvm = GVManager::instance();
        double limit = gvm.getNumeric("GENERATE_VELOCITY_LIMIT")/60/60;
        if (limit >= 0.0 && _velocity > limit)
        {
            _velocity = limit;
        }
    }
    assert(check);
    return check;
}

//======================================================================
double Vehicle::length() const
{
    return _length;
}

//======================================================================
double Vehicle::oldLength() const
{
    return _oldLength;
}

//======================================================================
double Vehicle::totalLength() const
{
    return _totalLength;
}

//======================================================================
double Vehicle::tripLength() const
{
    return _tripLength;
}

//======================================================================
double Vehicle::x() const
{
    AmuVector pv(_lane->beginConnector()->point(),
                 _lane->endConnector()->point());
    pv.normalize();

    double x = _lane->beginConnector()->point().x()+ _length*pv.x();

    pv.revoltXY(M_PI_2);
    x += _error * pv.x();

    return x;
}

//======================================================================
double Vehicle::y() const
{
    AmuVector pv(_lane->beginConnector()->point(),
                 _lane->endConnector()->point());
    pv.normalize();

    double y = _lane->beginConnector()->point().y()+ _length*pv.y();

    pv.revoltXY(M_PI_2);
    y += _error * pv.y();

    return y;
}

//======================================================================
double Vehicle::z() const
{
    AmuVector pv(_lane->beginConnector()->point(),
                 _lane->endConnector()->point());
    pv.normalize();

    double z = _lane->beginConnector()->point().z()+ _length*pv.z();

    pv.revoltXY(M_PI_2);
    z += _error * pv.z();

    return z;
}

//======================================================================
LaneBundle* Vehicle::laneBundle() const
{
    assert((!_section && _intersection)
           || (_section && !_intersection));

    if (_section)
    {
        return _section;
    }
    else if (_intersection)
    {
        return _intersection;
    }
    else
    {
        return NULL;
    }
}

//======================================================================
Section* Vehicle::section() const
{
    return _section;
}

//======================================================================
Intersection* Vehicle::intersection() const
{
    return _intersection;
}

//======================================================================
Lane* Vehicle::lane() const
{
    return _lane;
}

//======================================================================
bool Vehicle::isAwayFromOriginNode() const
{
    bool check = true;
    GVManager& gvm = GVManager::instance();

    if (_route->lastPassedIntersectionIndex()==0
        && _route->start()==_router->start()
        && _totalLength
        < gvm.getNumeric("NO_OUTPUT_LENGTH_FROM_ORIGIN_NODE"))
    {
        check = false;
    }
    return check;
}

//======================================================================
bool Vehicle::setLane(Intersection* inter, Lane* lane, double length)
{
    bool result = false;
    assert(_roadMap!=NULL && inter!=NULL && lane!=NULL);
    if (inter->isMyLane(lane))
    {
        _lane = lane;
        _decideNextLane(inter, lane);
        result = true;
    }
    return result;
}

//======================================================================
bool Vehicle::setLane(Section* section, Lane* lane, double length)
{
    bool result = false;
    assert(_roadMap!=NULL && section!=NULL && lane!=NULL);
    if (section->isMyLane(lane))
    {
        _localRouter.clear();
        _localRouter.localReroute(section, lane, length);
        _lane = lane;
        _decideNextLane(section, lane);
        assert(_nextLane);
        result = true;
    }
    return result;
}

//======================================================================
double Vehicle::velocity() const
{
    return _velocity;
}

//======================================================================
double Vehicle::accel() const
{
    return _accel;
}

//======================================================================
const AmuVector Vehicle::directionVector() const
{
    assert(_lane!=NULL);
    return _lane->directionVector();
}

//======================================================================
void Vehicle::notify()
{
#ifdef _OPENMP
    assert(!_running);
#endif
    if (_section!=NULL)
    {
        _section->addWatchedVehicle(this);
    }
    else
    {
        _intersection->addWatchedVehicle(this);
    }
    _isNotifying = true;
}

//======================================================================
void Vehicle::unnotify()
{
#ifndef _OPENMP
    if (_section!=NULL)
    {
        _section->eraseWatchedVehicle(this);
    }
    else
    {
        _intersection->eraseWatchedVehicle(this);
    }
#else
    if (_section!=NULL)
    {
        if (!_running)
        {
            _section->eraseWatchedVehicle(this);
        }
        else
        {
            _changeWatchedLane = CWLT_ERASE_SECTION;
            _changeWatchedLaneSection = _section;
        }
    }
    else
    {
        if (!_running)
        {
            _intersection->eraseWatchedVehicle(this);
        }
        else
        {
            _changeWatchedLane = CWLT_ERASE_INTERSECTION;
            _changeWatchedLaneIntersection = _intersection;
        }
    }
#endif
    _isNotifying = false; 
}

//======================================================================
Blinker Vehicle::blinker() const
{
    return _blinker;
}

//======================================================================
int Vehicle::directionFrom() const
{
    Intersection* inter;
    if (_intersection)
    {
        inter = _intersection;
    }
    else
    {
        inter = _section->intersection(_section->isUp(_lane));
    }
    if (inter)
    {
        const vector<Lane*>* liInter = _localRoute.lanesInIntersection();
        assert(inter->isMyLane((*liInter)[0]));
        return inter->direction((*liInter)[0]->beginConnector());
    }
    else 
    {
        return -1;
    }
}

//======================================================================
int Vehicle::directionTo() const
{
    Intersection* inter;
    if (_intersection)
    {
        inter = _intersection;
    }
    else
    {
        inter = _section->intersection(_section->isUp(_lane));
    }
    if (inter)
    {
        const vector<Lane*>* liInter = _localRoute.lanesInIntersection();
        assert(inter->isMyLane((*liInter)[liInter->size()-1]));
        return inter
            ->direction((*liInter)[liInter->size()-1]->endConnector());
    }
    else
    { 
        return -1;
    }
}

//======================================================================
VehicleShift& Vehicle::shiftLane()
{
    return _shiftLane;
}

//======================================================================
bool Vehicle::isSleep() const
{
    return (_sleepTime>0);
}

//======================================================================
void Vehicle::setStartTime(ulint startTime)
{
    _startTime = startTime;
}

//======================================================================
ulint Vehicle::startTime() const
{
    return _startTime;
}

//======================================================================
const vector<VirtualLeader*>* Vehicle::virtualLeaders() const
{
    return &_leaders;
}

//======================================================================
bool Vehicle::_isVisible(Vehicle* other) const
{
    /* この関数内で見通し計算についての関数を呼び出す */

    //--------------------------------------------------------------------
    // 一般のシミュレータは他車を正しく認知できるため
    // この関数は常にtrueを返す
    return true;

    //--------------------------------------------------------------------
    // サンプル: 自分と相手の位置を引数としての関数hoge()を呼ぶ
    // （hogeは別途定義）
    // return hoge(x(),y(),z(),other->x(),other->y(),other->z());
}

//======================================================================
bool Vehicle::_isYielding(Intersection* inter,
                          int thisDir,
                          int thatDir,
                          RelativeDirection turning,
                          Vehicle* other)
{
    /*
     * この段階では交錯単路の先頭車両を取得しただけなので，
     * 車両同士が本当に交錯するかどうかは不明であることに注意．
     *
     * これを解決するためには車両同士の
     * mainLaneが交差するかどうかを判断すればよいが
     * 実際に車両間の意思疎通はウインカーを通じて行うため，
     * 相手のmainLaneまで正確に取得するのは現実的ではない．
     * 以下のコードでは進入方向とウインカーの状態を元にして判断している．
     */

    AmuPoint crossPoint;

    // 交錯を厳密に評価、交錯点上に車がいたら相手側が避ける
    if (_strictCollisionCheck &&
        _lane->createIntersectionPoint(other->lane()->lineSegment(),
                                       &crossPoint))
    {
        AmuPoint thisPoint(x(), y(), z());
        AmuPoint thatPoint(other->x(), other->y(), other->z());
        double thisDistance = crossPoint.distance(thisPoint)
            - _bodyLength / 2;
        double thatDistance = crossPoint.distance(thatPoint)
            - other->bodyLength() / 2;
        if (thisDistance < 0 || thatDistance < 0)
        {
            if (thisDistance > thatDistance)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // スリープ状態の自動車は無視（自分の優先度が高い）
    if (other->isSleep())
    {
        return false;
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 交差点内での進路が交わらない場合は無視
    bool hasCrossPoint = false;
    const vector<Lane*>* thisLanes = _localRoute.lanesInIntersection();
    const vector<Lane*>* thatLanes = other->lanesInIntersection();
    for (unsigned int i=0; i<thisLanes->size(); i++)
    {
        for (unsigned int j=0; j<thatLanes->size(); j++)
        {
            if ((*thisLanes)[i]
                ->createIntersectionPoint((*thatLanes)[j]->lineSegment(),
                                          &crossPoint))
            {
                hasCrossPoint = true;
                break;
            }
        }
        if (hasCrossPoint==true) 
        {
            break;
        }
    }
    if (!hasCrossPoint)
    {
        return false;
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // 赤点滅の優先順位は最下位
    if (inter->signal()!=NULL)
    {
        // 自分が赤点滅でなく、相手が赤点滅の場合は無視
        if (inter->signal()->hasPermission(thisDir)!=3
            && inter->signal()->hasPermission(thatDir)==3)
        {
            return false;
        }
        // 自分が赤点滅で、相手が赤点滅でなければ必ず譲る
        if (inter->signal()->hasPermission(thisDir)==3
            && inter->signal()->hasPermission(thatDir)!=3)
        {
            return true;
        }
    }

    bool result = false;

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // thisDir==thatDirの場合は別の優先順位判定とする
    if (thisDir == thatDir)
    {
        // 直進優先
        if (turning == RD_STRAIGHT)
        {
            // 処理は必要ない 
        }
        // 左折は第2位
        // 自分より左の車線を直進する車両がいれば譲る
        else if (turning == RD_LEFT)
        {
            if (other->blinker().isNone())
            {
                // 自分と相手を結ぶベクトルを作成し、
                // 自分の進行方向となす角を求める
                /**
                 * 曲線レーンを作成する場合は要修正
                 */
                AmuVector posVector(AmuPoint(x(),y(),z()),
                                    AmuPoint(other->x(),
                                             other->y(),other->z()));
                if (directionVector().calcAngle(posVector)<0)
                {
                    result = true;
                }
            }
        }
        // 右折は第3位
        // 自分より右の車線を直進および左折する車両がいれば譲る
        else if (turning == RD_RIGHT)
        {
            if (other->blinker().isNone()
                || other->blinker().isLeft())
            {
                // 自分と相手を結ぶベクトルを作成し、
                // 自分の進行方向となす角を求める
                /**
                 * 曲線レーンを作成する場合は要修正
                 */
                AmuVector posVector(AmuPoint(x(),y(),z()),
                                    AmuPoint(other->x(),
                                             other->y(),other->z()));
                if (directionVector().calcAngle(posVector)>0)
                {
                    result = true;
                }
            }
        }
        // その他？
        else
        {
            result = true;
        }
    }

    //++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
    // thisDir!=thatDir 通常の優先順位の判定
    else {
        // 道路交通法第三十七条
        /*
         * 車両等は，交差点で右折する場合において，
         * 当該交差点において直進し，
         * 又は左折しようとする車両等があるときは，
         * 当該車両等の進行妨害をしては成らない． 
         *
         */
        if (turning == RD_RIGHT){
            if (other->blinker().isNone())
            {
                // 自分->右折 相手->直進
                result = true;
            }
            else if (other->blinker().isLeft())
            {
                // 自分->右折 相手->左折
                if (inter->relativeDirection(thisDir, thatDir)
                    ==(RD_RIGHT|RD_LEFT))
                {
                    // 相手が右から進入する場合は交錯しない
                    // 相手が左から進入する場合も交錯しないはず
                    result = false;
                }
                else
                {
                    result = true;
                }
            }
            else if (other->blinker().isRight()) {
                // 自分->右折 相手->右折
                if (inter->relativeDirection(thisDir, thatDir)==RD_LEFT)
                {
                    // 相手が左から進入する場合のみ譲る（左方優先）
                    result = true;
                }
                else
                {
                    result = false;
                }
            }
        }
    
        // 道路交通法第三十六条2
        /*
         * 車両等は，交通整理の行なわれていない交差点においては，
         * その通行している道路が優先道路である場合を除き，
         * 交差道路が優先道路であるとき、
         * 又はその通行している道路の幅員よりも
         * 交差道路の幅員が明らかに広いものであるときは、
         * 当該交差道路を通行する車両等の進行妨害をしてはならない。
         */
        else if (inter->numIn(thisDir)+inter->numOut(thisDir)
                 > inter->numIn(thatDir)+inter->numOut(thatDir)+4)
        {
            // 本来は厳密に道幅を定義すべき
            result = false;
        }
        else if (inter->numIn(thisDir)+inter->numOut(thisDir)+4
                 < inter->numIn(thatDir)+inter->numOut(thatDir))
        {
            // 本来は厳密に道幅を定義すべき
            result = true;
        }
    
        // 道路交通法第三十六条1
        /*
         * 車両等は，交通整理の行なわれていない交差点においては，
         * 次項の規定が適用される場合を除き、次の各号に掲げる区分に従い，
         * 当該各号に掲げる車両等の進行妨害をしてはならない．
         * 一 車両である場合
         *    交差道路を左方から進行してくる車両及び
         *    交差道路を通行する路面電車
         * 二 路面電車である場合
         *    交差道路を左方から進行してくる路面電車
         *
         * (修正...法律的に正しいかは不明)
         * 「左方から進行」ではなく，「左方から直進」とする．
         * これにより直進優先が左方優先に勝る．
         */
        else if (inter->relativeDirection(thisDir, thatDir)==RD_LEFT)
        {
            if (other->blinker().isNone() && 
                turning!=RD_LEFT)
            {
                // 自分->直進or左折 相手->直進
                result = true;
            }
            else
            {
                result = false;
            }
        }

        // その他、直進優先（法律には明記されていない）
        else
        {
            if (other->blinker().isNone() && turning!=RD_STRAIGHT)
            {
                result = true;
            }
            else
            {
                result =  false;
            }
        }
    }

    return result;
}

//======================================================================
void Vehicle::_decideNextLane(Intersection* inter, Lane* lane)
{
    // 次のレーン候補の数
    int num = inter->numNextLane(lane);

    if (num!=0)
    {
        /*
         * LocalRouteは交差点を出るまでの経路を持っている
         * 正確には，交差点を出て次の単路に入った
         * 最初のレーンまでを保持している
         */
        if (_localRouter.isSearched())
        {
            _nextLane = _localRoute.next(lane);
        }
        else
        {
            _nextLane = inter->mostStraightNextLane(lane);
        }
        assert(_nextLane);
    }
    else
    {
        // この後のrerouteに任せる
    }
}

//======================================================================
void Vehicle::_decideNextLane(Section* section, Lane* lane)
{
    // 次のレーン候補の数
    int num = section->numNextLane(lane);
    if (num!=0)
    {
        if (section->isNextLaneMine(lane))
        {
            if(_localRouter.isSearched())
            {
                _nextLane = _localRoute.next(lane);
                assert(_nextLane != NULL);
            }
            else
            {
                _nextLane = section->mostStraightNextLane(lane);
            }
        }
        else
        {
            if(_localRouter.isSearched())
            {
                _nextLane = _localRoute.next(lane);
            }
            else
            {
                _nextLane = section->mostStraightNextLane(lane);
            }
        }
        assert(_nextLane != NULL);
    }
}

//======================================================================
void Vehicle::_runIntersection2Intersection()
{
    _lane = _nextLane;
    _decideNextLane(_intersection, _lane);
}

//======================================================================
void Vehicle::_runIntersection2Section()
{
    _section = _intersection->nextSection(_lane);
    assert(_section);

    // _intersectionに交差点通過時間を通知
    /// 並列なら保存して非並列で後処理
    int from = _intersection->direction(_prevIntersection);
    int to   = _intersection->direction(_section);
    assert(0<=from && from<_intersection->numNext());
    assert(0<=to && to<_intersection->numNext());
#ifndef _OPENMP
    _intersection->addPassTime(from, to,
                               TimeManager::time()-_entryTime);
#else
    _passTimeIntersection = _intersection;
    _passTimeFrom = from;
    _passTimeTo = to;
    _passTime = TimeManager::time()-_entryTime;
#endif
    _entryTime = TimeManager::time();

    // _watchedVehiclesの登録状況の受渡し
    /*
     * 本来は車線変更中にレーン束を移ることはないはずだが...
     */
    if (_isNotifying)
    {
#ifndef _OPENMP
        _intersection->eraseWatchedVehicle(this);
        _section->addWatchedVehicle(this);
#else
        // 注目（車線変更）車両レーン変更設定
        _changeWatchedLane = CWLT_INTERSECTION_TO_SECTION;
        _changeWatchedLaneIntersection = _intersection;
        _changeWatchedLaneSection = _section;
#endif
    }
  
    // 車線変更に失敗したとき場合などのために
    // 必要であればここでルートを再探索する。
    if(_prevIntersection != NULL)
    {
        Intersection* next
            = _intersection->next(_intersection->direction(_section));
        if (dynamic_cast<ODNode*>(next)==NULL
            && _route->next(_intersection, next)==NULL)
        {
            reroute(_section, _intersection);
        }
    }

    _prevIntersection = _intersection;
    _intersection=NULL;
    _lane = _nextLane;
    _localRouter.clear();
    _localRouter.localReroute(_section, _lane, _length);
    _decideNextLane(_section, _lane);
}

//======================================================================
void Vehicle::_runSection2Section()
{
    _lane = _nextLane;
    if (!_localRouter.isSearched())
    {
        _localRouter.clear();
        _localRouter.localReroute(_section, _lane, _length);
    }
    _decideNextLane(_section, _lane);
}

//======================================================================
void Vehicle::_runSection2Intersection()
{
    _intersection = _section->nextIntersection(_lane);
    assert(_intersection);

    // 車線変更を行いながら交差点に入るとMATESが落ちるのでここでトラップ．
    // とりあえず車線変更を緊急中断
    if (_shiftLane.isActive())
    {
        cerr << "WARNING: would enter intersection while lane shifting."
             << endl
             << "vehicle:" << _id 
             << " in section:" << _section->id() << endl;
        _shiftLane.abortShift();
    }

    //最後に通過した交差点として新しい交差点を登録。
    if(_prevIntersection == NULL)
    {
        _route->setLastPassedIntersection(_intersection);
    }
    else
    {
        _route->setLastPassedIntersection(_prevIntersection,_intersection);
    }
    //最後に通過した経由地として新しい交差点を登録。
    _router->setLastPassedStopPoint(_intersection->id());

    //_watchedVehiclesの登録状況の受渡し
    /*
     * 本来は車線変更中にレーン束を移ることはないはずだが...
     */
    if (_isNotifying)
    {
#ifndef _OPENMP
        _section->eraseWatchedVehicle(this);
        if (dynamic_cast<ODNode*>(_intersection)==NULL)
        {
            // ODノードに入る(次のステップで消える)ときにはaddしない
            _intersection->addWatchedVehicle(this);
        }
#else
        // 注目（車線変更）車両レーン変更設定
        _changeWatchedLane = CWLT_SECTION_TO_INTERSECTION;
        _changeWatchedLaneSection = _section;
        _changeWatchedLaneIntersection = _intersection;
#endif
    }

    _section = NULL;
    _lane = _nextLane;
    // ODノードでなければ次のレーンを検索
    if (dynamic_cast<ODNode*>(_intersection)==NULL)
    {
        _decideNextLane(_intersection, _lane);
    }
}

//======================================================================
const Route* Vehicle::route() const
{
    return _route;
}

//======================================================================
ARouter* Vehicle::router()
{
    return _router;
}

//======================================================================
/*
 * この関数は以前は全ての再探索で使用されていたが，
 * 現在は車両発生時にしか呼ばれなくなった．
 * ("reroute"という関数名は正しくない)
 */
bool Vehicle::reroute(const Intersection* start)
{
    bool succeed = true;
    int _routerIStep = 10000;
    Route* new_route = NULL;

#ifdef _OPENMP
    _routerIStep = _routerSearchStep;
    _rerouteCnt++;
#endif

    _router->search(start, _routerIStep, new_route);

    if(new_route == NULL)
    {
        succeed = false;
    }
    else if(new_route->size() == 0)
    {
        succeed = false;
    }

    if(_route != NULL)
    {
        delete _route;
    }

    if(succeed)
    {
        _route = new_route;
        _localRouter.setRoute(_route);
    }
    else
    {
        if(new_route != NULL)
        {
            delete new_route;
            new_route = NULL;
        }
        // ルート探索失敗した場合は仕方が無いのでスタート地点とゴール地点を
        // 直接結ぶ経路を設定する (ほとんどの場合、このような経路はない。)
        // 落ちるのを防ぐための苦肉の策
        _route = new Route();
        _route->push(const_cast<Intersection*>(_router->start()));
        _route->push(const_cast<Intersection*>(_router->goal()));
        _localRouter.setRoute(_route);
    }

    return succeed;
}

//======================================================================
bool Vehicle::reroute(const Section* section,
                      const Intersection* start)
{
    bool succeed = true;
    int _routerIStep = 10000;
    Route* new_route = NULL;

#ifdef _OPENMP
    _routerIStep = _routerSearchStep;
    _rerouteCnt++;
#endif

    // cout << "vehicle:" << _id << " reroute." << endl;
    _router->search(section, start, _routerIStep, new_route);

    if(new_route == NULL)
    {
        succeed = false;
    }
    else if(new_route->size() == 0)
    {
        succeed = false;
    }

    // 古い経路を削除
    if(_route != NULL)
    {
        delete _route;
    }

    // 新しい経路を設定
    if(succeed)
    {
        _route = new_route;
        _localRouter.setRoute(_route);
    }
    else
    {
        if(new_route != NULL)
        {
            delete new_route;
            new_route = NULL;
        }
        // ルート探索失敗した場合は仕方が無いのでスタート地点とゴール地点を
        // 直接結ぶ経路を設定する(ほとんどの場合、このような経路はない。)
        // 落ちるのを防ぐための苦肉の策
        _route = new Route();
        _route->push(const_cast<Intersection*>(_router->start()));
        _route->push(const_cast<Intersection*>(_router->goal()));
        _localRouter.setRoute(_route);
    }

    return succeed;
}

//======================================================================
const vector<Lane*>* Vehicle::lanesInIntersection() const
{
    return _localRoute.lanesInIntersection();
}

#ifdef USE_ADDIN
//======================================================================
double Vehicle::vMax() const
{
    return _vMax;
}

//======================================================================
bool Vehicle::checkSetAddinRoute(vector<string>* idList)
{
    int i;
    vector<Intersection*> intersections;

    // 経路到着可能チェック、アドイン呼び元ではできない
    for (i = 0; i < idList->size(); i++)
    {
        intersections.push_back(_roadMap->intersection(idList->at(i)));
    }
    for (i = 1; i < idList->size() - 1; i++)
    {
        if (!intersections[i]->isReachable(intersections[i - 1], intersections[i + 1]))
        {
            return false;
        }
    }
    return true;
}

//======================================================================
void Vehicle::setAddinRoute(vector<string>* idList, bool newRoute)
{
    int lastIndex, i, size;
    Intersection* lastIntersection;

    // アドインから経路設定
    lastIndex = _route->lastPassedIntersectionIndex();
    lastIntersection = _route->route()->at(lastIndex);
    if (newRoute)
    {
        delete _route;
        _route = new Route();
    }
    else
    {
        size = _route->route()->size();
        for (i = size - 1; i >= lastIndex; i--)
            _route->pop();
    }
    for (i = 0; i < idList->size(); i++)
    {
        _route->push(_roadMap->intersection(idList->at(i)));
    }
    _route->setLastPassedIntersection(lastIntersection);

    if (newRoute)
    {
        _localRouter.setRoute(_route);
    }
    _localRouter.clear();
    _localRouter.localReroute(_section, _lane, _length);
    _decideNextLane(_section, _lane);
}

//======================================================================
void* Vehicle::createAddinData(unsigned int size)
{
    if (_addinData != NULL)
    {
        free(_addinData);
    }
    _addinData = malloc(size);
    return _addinData;
}

//======================================================================
void* Vehicle::getAddinData()
{
    return _addinData;
}
#endif

//======================================================================
void Vehicle::print() const
{
    cout << "--- Vehicle Information ---" << endl;
    cout << "ID: " << _id << ", Type: " << _type << endl;

    // 位置と速度に関するもの
    if (_section!=NULL)
    {
        cout << "Section ID, Lane ID: " << _section->id();
    }
    else
    {
        cout << "Intersection ID, Lane ID: " << _intersection->id();
    }
    cout << ", " << _lane->id() << endl;
    cout << "Length, Error: " << _length << ", " << _error << endl;
    cout << "Velocity, ErrorVelocity: "
         << _velocity <<", "<< _errorVelocity << endl;
    cout << "(x,y,z)=("
         << x() << ", " << y() << ", " << z() << ")"<< endl;

    // 経路に関するもの
    _router->printParam();
    _route->print(cout);
    _localRoute.print();

    // 交錯レーンに関するもの
    if (_section!=NULL)
    {
        Intersection* nextInter
            = _section->intersection(_section->isUp(_lane));
        if (nextInter)
        {
            vector<Lane*> cli;
            vector<Lane*> cls;
            nextInter->collisionLanes(_localRoute.lanesInIntersection(),
                                      &cli, &cls);
            cout << "Collision Lanes in Intersection:" << endl;
            for (unsigned int i=0; i<cli.size(); i++)
            {
                cout << "\t" << cli[i]->id() << endl;
            }
            cout << "Collision Lanes in Section:" << endl;
            for (unsigned int i=0; i<cls.size(); i++)
            {
                cout << "\t" << cls[i]->id() << " of section "
                     << nextInter->nextSection
                    (nextInter->direction
                     (cls[i]->endConnector()))->id() 
                     << endl;
            }
        }
    }
 
    // 車線変更に関するもの
    if (_shiftLane.isActive())
    {
        cout << "Shift Lane:" << endl;
        cout << "  Target Lane  : " << _shiftLane.laneTo()->id() << endl;
        cout << "  Target Length: " << _shiftLane.lengthTo() << endl;
    }

#ifdef VL_DEBUG
    cout << "Virtual Leaders:" << endl;
    for (unsigned int i=0; i<_leaders.size(); i++)
    {
        _leaders[i]->print();
    }
#endif //VL_DEBUG

    cout << endl;
}

#ifdef _OPENMP
//======================================================================
ulint Vehicle::genIntensiveStep() const
{
    return _genIntensiveStep;
}

//======================================================================
void Vehicle::setGenIntensiveStep(ulint genIntensiveStep)
{
    _genIntensiveStep = genIntensiveStep;
}

//======================================================================
int Vehicle::rerouteCnt() const
{
    return _rerouteCnt;
}

//======================================================================
void Vehicle::clearRerouteCnt()
{
    _rerouteCnt = 0;
}
#endif
